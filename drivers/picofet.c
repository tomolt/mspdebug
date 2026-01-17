/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2025-2026 Thomas Oltmann
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * PicoFET is an open-source firmware for the Raspberry Pi Pico and compatible
 * boards that turns them into debuggers & programmers for TI MSP430 family MCUs.
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include "picofet.h"
#include "cdc_acm.h"
#include "util.h"
#include "output.h"
#include "ctrlc.h"

#define MIN(a,b) ((a)<(b)?(a):(b))

#define PSEUDO_STATUS_IO_ERROR 600

#define BUFFER_SIZE 256

struct pfet {
	struct device device;
	transport_t   tran;
	char         *buffer;
	unsigned      buffered;
	address_t     mcu_id;
};

/* Sends a number of bytes to the PicoFET device.
 */
static bool send_message(struct pfet *pfet, const void *data, address_t length)
{
	int ret = pfet->tran->ops->send(pfet->tran, data, length);
	if (ret < 0) {
		printc_err("picofet: encountered I/O error.\n");
		return false;
	}
	return true;
}

/* Waits until we have received some bytes from the PicoFET device,
 * and stores them in the (input) buffer in the pfet struct.
 */
static bool wait_for_input(struct pfet *pfet)
{
	int ret = pfet->tran->ops->recv(pfet->tran,
		(uint8_t *)pfet->buffer + pfet->buffered,
		BUFFER_SIZE - pfet->buffered);
	if (ret < 0) {
		printc_err("picofet: encountered I/O error.\n");
		return false;
	}
	pfet->buffered += ret;
	return true;
}

/* Waits until we have received a whole line (delimited by CR LF) from the PicoFET device.
 * The line delimiter bytes are overwritten in-place with NUL characters.
 * On success, returns the number of bytes in the line (including the line delimiters).
 * On I/O failure, returns a negative value.
 */
static int wait_for_line(struct pfet *pfet)
{
	char *lf;
	bool ok;

	while (!(lf = memchr(pfet->buffer, '\n', pfet->buffered))) {
		if (pfet->buffered == BUFFER_SIZE) {
			return -1;
		}

		ok = wait_for_input(pfet);
		if (!ok) return -1;
	}

	*lf = '\0';
	if (lf > pfet->buffer && *(lf-1) == '\r') {
		*(lf-1) = '\0';
	}

	return lf + 1 - pfet->buffer;
}

static void discard_input(struct pfet *pfet, int num_bytes)
{
	assert(num_bytes >= 0);
	pfet->buffered -= num_bytes;
	memmove(pfet->buffer, pfet->buffer + num_bytes, pfet->buffered);
}

/* Receives a status code.
 * Status codes are three-digit numbers akin to FTP status codes.
 * This function only returns true if a status code was received and the code indicates success.
 */
static bool recv_status(struct pfet *pfet, int *out_status)
{
	unsigned long status;
	int input_length;
	bool success;

	input_length = wait_for_line(pfet);
	if (input_length < 0) {
		if (out_status) {
			*out_status = PSEUDO_STATUS_IO_ERROR;
		}
		return false;
	}
	status = strtoul(pfet->buffer, NULL, 10);
	if (out_status) {
		*out_status = status;
	}
	// We consider 1xx/2xx/3xx as success, 4xx/5xx as failure.
	success = (status >= 100) && (status < 400);
	if (!success) {
		printc_err("picofet: %s\n", pfet->buffer);
	}
	discard_input(pfet, input_length);
	return success;
}

/* Receives an address/machine word/integer from the PicoFET device.
 * These are formatted as a line of text containing a single (hexadecimal) human-readable integer.
 */
static bool recv_address(struct pfet *pfet, address_t *out_addr)
{
	int input_length;

	input_length = wait_for_line(pfet);
	if (input_length < 0) {
		printc_err("picofet: I/O error\n");
		*out_addr = 0;
		return false;
	}
	if (out_addr) {
		*out_addr = strtoul(pfet->buffer, NULL, 0);
	}
	discard_input(pfet, input_length);
	return true;
}

/* Sends a human-readable command to the PicoFET device, and receive the resulting status code.
 * The command can be formatted with standard printf() format specifiers.
 * This function optionally stores the received status in *out_status.
 * It only returns true if the command was completed and returned a successful status code.
 */
static bool do_command(struct pfet *pfet, int *out_status, const char *format, ...)
{
	va_list va;
	int length;
	bool ok;

	// If we return before parsing the status code bc of an error, this is the code we'll return instead.
	if (out_status) {
		*out_status = PSEUDO_STATUS_IO_ERROR;
	}

	if (pfet->buffered) {
		// Theoretically, this should not happen, as we operate in half-duplex mode.
		printc_dbg("picofet: warning: discarding I/O buffer contents without looking at them.\n");
	}
	pfet->buffered = 0;

	va_start(va, format);
	length = vsprintf(pfet->buffer, format, va);
	if (length < 0) return false;
	va_end(va);

	printc_dbg("picofet: do_command: %.*s", length, pfet->buffer);

	ok = send_message(pfet, pfet->buffer, length);
	if (!ok) return false;

	return recv_status(pfet, out_status);
}

static bool init_pfet(struct pfet *pfet)
{
	bool ok;

	// Set DTR, otherwise USB device wouldn't recognize that the connection has been established.
	ok = pfet->tran->ops->set_modem(pfet->tran, TRANSPORT_MODEM_DTR) >= 0;
	if (!ok) return false;

	ok = do_command(pfet, NULL, "MCU:ATTACH\r\n");
	if (!ok) return false;
	ok = recv_address(pfet, NULL);
	if (!ok) return false;

	ok = do_command(pfet, NULL, "MCU:GET_ID\r\n");
	if (!ok) return false;
	ok = recv_address(pfet, &pfet->mcu_id);
	if (!ok) return false;

	printc("picofet: attached to mcu 0x%"PRIx32"\n", pfet->mcu_id);

	return true;
}

static void deinit_pfet(struct pfet *pfet)
{
	bool ok;
	
	ok = do_command(pfet, NULL, "MCU:DETACH 0x%"PRIx32"\r\n", pfet->mcu_id);
	if (!ok) return;
}

static bool read_range(struct pfet *pfet, address_t addr, uint8_t *mem, address_t len)
{
	address_t cursor;
	bool ok;

	ok = do_command(pfet, NULL, "RAM:READ 0 0x%"PRIx32 " %"PRIu32"\r\n", addr, len);
	if (!ok) return false;

	ok = do_command(pfet, NULL, "BUF:DOWNLOAD_BIN 0 %"PRIu32"\r\n", len);
	if (!ok) return false;

	// Keep receiving bytes until we have downloaded the whole answer
	cursor = 0;
	for (;;) {
		address_t step_size = MIN(len - cursor, pfet->buffered);
		memcpy(mem + cursor, pfet->buffer, step_size);
		cursor += step_size;
		discard_input(pfet, step_size);
		if (cursor == len) break;

		ok = wait_for_input(pfet);
		if (!ok) return false;
	}

	return recv_status(pfet, NULL);
}

static bool write_range(struct pfet *pfet, address_t addr, const uint8_t *mem, address_t num_bytes, const struct chipinfo_memory *meminfo)
{
	bool ok;

	ok = do_command(pfet, NULL, "BUF:UPLOAD_BIN 0 %" PRIu32 "\r\n", num_bytes);
	if (!ok) return false;

	ok = send_message(pfet, mem, num_bytes);
	if (!ok) return false;

	ok = recv_status(pfet, NULL);
	if (!ok) return false;

	switch (meminfo->type) {
	case CHIPINFO_MEMTYPE_RAM:
		ok = do_command(pfet, NULL, "RAM:WRITE 0 0x%"PRIx32" %"PRIu32"\r\n", addr, num_bytes);
		return ok;

	case CHIPINFO_MEMTYPE_FLASH:
		ok = do_command(pfet, NULL, "FLASH:WRITE 0 0x%"PRIx32" %"PRIu32"\r\n", addr, num_bytes);
		return ok;

	default:
		printc_err("picofet: Attempting to write to memory range that isn't RAM or FLASH.\n");
		return false;
	}
}

static device_t pfet_open(const struct device_args *args)
{
	struct pfet *pfet;
	transport_t tran;

	if (args->flags & DEVICE_FLAG_TTY) {
		printc_err("picofet: this driver only supports USB transport\n");
		return NULL;
	}

	if (!(args->flags & DEVICE_FLAG_JTAG)) {
		printc_err("picofet: this driver does not support Spy-Bi-Wire\n");
		return NULL;
	}

	// Both RP2040 and RP2350 MCUs are supported, and they differ in USB product id,
	// so we have to search for either id.
	tran = cdc_acm_open(args->path, args->requested_serial, 115200, 0x2E8A, 0x0009);
	if (!tran) {
		tran = cdc_acm_open(args->path, args->requested_serial, 115200, 0x2E8A, 0x000A);
	}

	if (!tran) {
		printc_err("picofet: failed to establish USB CDC-ACM transport\n");
		return NULL;
	}

	pfet = malloc(sizeof *pfet);
	if (!pfet) {
		printc_err("picofet: malloc: %s\n", last_error());
		tran->ops->destroy(tran);
		return NULL;
	}

	memset(pfet, 0, sizeof *pfet);
	pfet->device.type = &device_picofet;
	pfet->device.max_breakpoints = 2; // supported by all devices
	pfet->device.need_probe = 1;
	pfet->tran = tran;

	pfet->buffer = malloc(BUFFER_SIZE);
	if (!pfet->buffer) {
		printc_err("picofet: malloc: %s\n", last_error());
		tran->ops->destroy(tran);
		free(pfet);
		return NULL;
	}

	if (!init_pfet(pfet)) {
		tran->ops->destroy(tran);
		free(pfet->buffer);
		free(pfet);
		return NULL;
	}

	return &pfet->device;
}

static void pfet_destroy(device_t dev)
{
	struct pfet *pfet = (struct pfet *)dev;
	if (!pfet) return;

	deinit_pfet(pfet);
	pfet->tran->ops->destroy(pfet->tran);
	free(pfet->buffer);
	free(pfet);
}

static int pfet_readmem(device_t dev, address_t addr, uint8_t *mem, address_t num_bytes)
{ 
	struct pfet *pfet = (struct pfet *)dev;
	const struct chipinfo_memory *m;
	address_t cursor = 0, range_bytes;
	bool ok;

	while (cursor < num_bytes) {
		range_bytes = check_range(dev->chip, addr + cursor, num_bytes - cursor, &m);
		if (m == NULL) {
			printc_err("picofet: memory read out of range\n");
			return -1;
		}
		ok = read_range(pfet, addr + cursor, mem + cursor, range_bytes);
		if (!ok) return -1;
		cursor += range_bytes;
	}

	return 0;
}

static int pfet_writemem(device_t dev, address_t addr, const uint8_t *mem, address_t num_bytes)
{
	struct pfet *pfet = (struct pfet *)dev;
	const struct chipinfo_memory *m;
	address_t cursor = 0, range_bytes;
	bool ok;

	while (cursor < num_bytes) {
		range_bytes = check_range(dev->chip, addr + cursor, num_bytes - cursor, &m);
		if (m == NULL) {
			printc_err("picofet: memory write out of range.\n");
			return -1;
		}
		ok = write_range(pfet, addr + cursor, mem + cursor, range_bytes, m);
		if (!ok) return -1;
		cursor += range_bytes;
	}

	return 0;
}

static bool pfet_refresh_bps(device_t dev, struct pfet *pfet)
{
	int i;
	struct device_breakpoint *bp;
	address_t addr;
	bool ok, all_ok = true;

	for (i = 0; i < dev->max_breakpoints; i++) {
		bp = &dev->breakpoints[i];

		printc_dbg("picofet: refresh breakpoint %d: type=%d "
			   "addr=%04x flags=%04x\n",
			   i, bp->type, bp->addr, bp->flags);

		if ((bp->flags &  DEVICE_BP_DIRTY) &&
		    (bp->type  == DEVICE_BPTYPE_BREAK)) {
			addr = bp->addr;

			if (!(bp->flags & DEVICE_BP_ENABLED)) {
				addr = 0;
			}

			ok = do_command(pfet, NULL, "BREAK:SET %d 0x%"PRIx32"\r\n", i, addr);
			all_ok &= ok;

			if (ok) {
				bp->flags &= ~DEVICE_BP_DIRTY;
			} else {
				printc_err("picofet: failed to refresh "
					   "breakpoint #%d\n", i);
			}
		}
	}

	return all_ok;
}

static int pfet_erase(device_t dev, device_erase_type_t type, address_t address)
{
	struct pfet *pfet = (struct pfet *)dev;
	bool ok;

	switch (type) {
	case DEVICE_ERASE_ALL:
		ok = do_command(pfet, NULL, "FLASH:ERASE_ALL\r\n");
		break;
	case DEVICE_ERASE_MAIN:
		ok = do_command(pfet, NULL, "FLASH:ERASE_MAIN\r\n");
		break;
	case DEVICE_ERASE_SEGMENT:
		ok = do_command(pfet, NULL, "FLASH:ERASE_SEG 0x%"PRIx32"\r\n");
		break;
	default:
		printc_err("picofet: attempting unknown type of erase operation.\n");
		return -1;
	}

	return ok ? 0 : -1;
}

static int pfet_getregs(device_t dev, address_t *regs)
{
	struct pfet *pfet = (struct pfet *)dev;
	bool ok;

	memset(regs, 0, DEVICE_NUM_REGS * sizeof (*regs));
	for (int r = 0; r < DEVICE_NUM_REGS; r++) {
		ok = do_command(pfet, NULL, "REG:READ %d\r\n", r);
		if (!ok) return -1;
		ok = recv_address(pfet, &regs[r]);
		if (!ok) return -1;
	}

	return 0;
}

static int pfet_setregs(device_t dev, const address_t *regs)
{
	struct pfet *pfet = (struct pfet *)dev;
	bool ok;

	for (int r = 0; r < DEVICE_NUM_REGS; r++) {
		ok = do_command(pfet, NULL, "REG:WRITE %d 0x%"PRIx32"\r\n", r, regs[r]);
		if (!ok) return -1;
	}

	return 0;
}

static int pfet_ctl(device_t dev, device_ctl_t op)
{
	struct pfet *pfet = (struct pfet *)dev;
	bool ok;

	switch (op) {
	case DEVICE_CTL_RESET:
		ok = do_command(pfet, NULL, "MCU:RESET\r\n");
		break;

	case DEVICE_CTL_RUN:
		// Transfer changed breakpoints to device
		ok = pfet_refresh_bps(dev, pfet);
		if (!ok) return -1;
		ok = do_command(pfet, NULL, "MCU:CONTINUE\r\n");
		break;

	case DEVICE_CTL_HALT:
		ok = do_command(pfet, NULL, "MCU:HALT\r\n");
		break;

	case DEVICE_CTL_STEP:
		ok = do_command(pfet, NULL, "MCU:STEP\r\n");
		break;

	default:
		printc_err("picofet: operation is not supported.\n");
		return -1;
	}

	return ok ? 0 : -1;
}

static device_status_t pfet_poll(device_t dev)
{
	struct pfet *pfet = (struct pfet *)dev;
	address_t halted;
	bool ok;

	if (delay_ms(100) < 0 || ctrlc_check()) {
		return DEVICE_STATUS_INTR;
	}

	ok = do_command(pfet, NULL, "MCU:IS_HALTED\r\n");
	if (!ok) {
		return DEVICE_STATUS_ERROR;
	}
	ok = recv_address(pfet, &halted);
	if (!ok) {
		return DEVICE_STATUS_ERROR;
	}

	if (halted) {
		return DEVICE_STATUS_HALTED;
	} else {
		return DEVICE_STATUS_RUNNING;
	}
}

static int pfet_getconfigfuses(device_t dev)
{
	struct pfet *pfet = (struct pfet *)dev;
	address_t fuses;
	bool ok;

	ok = do_command(pfet, NULL, "FUSES:GET_CONFIG\r\n");
	if (!ok) return 0;
	ok = recv_address(pfet, &fuses);
	if (!ok) return 0;

	return fuses;
}

const struct device_class device_picofet = {
	.name     = "picofet",
	.help     = "Raspberry Pi Pico (or similar) as a FET",
	.open     = pfet_open,
	.destroy  = pfet_destroy,
	.readmem  = pfet_readmem,
	.writemem = pfet_writemem,
	.erase    = pfet_erase,
	.getregs  = pfet_getregs,
	.setregs  = pfet_setregs,
	.ctl      = pfet_ctl,
	.poll     = pfet_poll,
	.getconfigfuses = pfet_getconfigfuses,
};
