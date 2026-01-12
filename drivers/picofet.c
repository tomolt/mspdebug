/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2025-2026 Thomas Oltmann
 *
 * PicoFET is an open-source firmware for the Raspberry Pi Pico and compatible
 * MCUs that turns them into MSP430-compatible JTAG debug probes.
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
 */

#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "picofet.h"
#include "picofet_proto.h"

#include "cdc_acm.h"

#include "util.h"
#include "output.h"
#include "ctrlc.h"

#define MIN(a,b) ((a)<(b)?(a):(b))

#define BUFFER_SIZE 256

struct pfet {
	struct device device;
	transport_t   tran;
	char         *buffer;
	unsigned      buffered;
	address_t     mcu_id;
};

static char *wait_for_line(struct pfet *pfet)
{
	char *lf;
	int ret;

	while (!(lf = memchr(pfet->buffer, '\n', pfet->buffered))) {
		if (pfet->buffered == BUFFER_SIZE) {
			return NULL;
		}

		ret = pfet->tran->ops->recv(pfet->tran,
			(uint8_t *)pfet->buffer + pfet->buffered,
			BUFFER_SIZE - pfet->buffered);
		if (ret < 0) {
			return NULL;
		}

		pfet->buffered += ret;
	}

	*lf = '\0';
	if (lf > pfet->buffer && *(lf-1) == '\r') {
		*(lf-1) = '\0';
	}

	return lf;
}

static void discard_line(struct pfet *pfet, char *lf)
{
	pfet->buffered -= lf + 1 - pfet->buffer;
	memmove(pfet->buffer, lf + 1, pfet->buffered);
}

static bool recv_status(struct pfet *pfet, int *out_status)
{
	int status;
	char *lf;

	lf = wait_for_line(pfet);
	if (!lf) {
		printc_err("picofet: I/O error\n");
		*out_status = 900;
		return false;
	}
	status = atoi(pfet->buffer);
	if (out_status) {
		*out_status = status;
	}
	if (status >= 400) {
		printc_err("picofet: %s\n", pfet->buffer);
		discard_line(pfet, lf);
		return false;
	}
	discard_line(pfet, lf);
	return true;
}

static bool recv_address(struct pfet *pfet, address_t *out_addr)
{
	char *lf;

	lf = wait_for_line(pfet);
	if (!lf) {
		printc_err("picofet: I/O error\n");
		*out_addr = 0;
		return false;
	}
	if (out_addr) {
		*out_addr = strtoul(pfet->buffer, NULL, 0);
	}
	discard_line(pfet, lf);
	return true;
}

static int do_command(struct pfet *pfet, const char *format, ...)
{
	va_list va;
	int len, status;
	bool ok;

	if (pfet->buffered) {
		// Theoretically, this should not happen, as we operate in half-duplex mode.
		printc_dbg("picofet: warning: discarding I/O buffer contents without looking at them.\n");
	}
	pfet->buffered = 0;

	va_start(va, format);
	len = vsprintf(pfet->buffer, format, va);
	if (len < 0) {
		return 900;
	}
	va_end(va);

	printc_dbg("picofet: do_command: %.*s", len, pfet->buffer);

	ok = pfet->tran->ops->send(pfet->tran, (uint8_t *)pfet->buffer, len) >= 0;
	if (!ok) {
		return 900;
	}

	recv_status(pfet, &status);
	return status;
}

static bool init_pfet(struct pfet *pfet)
{
	int status;
	bool ok;

	ok = pfet->tran->ops->set_modem(pfet->tran, TRANSPORT_MODEM_DTR) >= 0;
	if (!ok) {
		return false;
	}

	status = do_command(pfet, "MCU:ATTACH\r\n");
	if (status != STATUS_OK) {
		return false;
	}
	ok = recv_address(pfet, NULL);
	if (!ok) {
		return false;
	}

	status = do_command(pfet, "MCU:GET_ID\r\n");
	if (status >= 400) {
		return -1;
	}
	ok = recv_address(pfet, &pfet->mcu_id);
	if (!ok) {
		return false;
	}
	printc("picofet: attached to mcu 0x%"PRIx32"\n", pfet->mcu_id);

	return true;
}

static void deinit_pfet(struct pfet *pfet)
{
	int status;
	
	status = do_command(pfet, "MCU:DETACH 0x%"PRIx32"\r\n", pfet->mcu_id);
	if (status != STATUS_OK) {
		return;
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
	//pfet->device.max_breakpoints = 2; // supported by all devices
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

static bool read_range(struct pfet *pfet, address_t addr, uint8_t *mem, address_t len)
{
	int status;

	status = do_command(pfet, "RAM:READ 0 0x%"PRIx32 " %"PRIu32"\r\n", addr, len);
	if (status >= 400) {
		return false;
	}

	status = do_command(pfet, "BUF:DOWNLOAD_BIN 0 %"PRIu32"\r\n", len);
	if (status >= 400) {
		return false;
	}

	// Keep receiving bytes until we have downloaded the whole answer
	address_t cursor = 0;
	for (;;) {
		address_t chunk = MIN(len - cursor, pfet->buffered);
		memcpy(mem + cursor, pfet->buffer, chunk);
		cursor += chunk;
		pfet->buffered -= chunk;
		memmove(pfet->buffer, pfet->buffer + chunk, pfet->buffered);
		if (cursor == len) {
			break;
		}

		int ret = pfet->tran->ops->recv(pfet->tran,
			(uint8_t *)pfet->buffer + pfet->buffered,
			BUFFER_SIZE - pfet->buffered);
		if (ret < 0) {
			return false;
		}

		pfet->buffered += ret;
	}

	return recv_status(pfet, NULL);
}

static int pfet_readmem(device_t dev, address_t addr, uint8_t *mem, address_t len)
{ 
	struct pfet *pfet = (struct pfet *)dev;
	const struct chipinfo_memory *m;
	address_t cursor = 0, rlen;
	bool ok;

	while (cursor < len) {
		rlen = check_range(dev->chip, addr + cursor, len - cursor, &m);
		if (m == NULL) {
			printc_err("picofet: memory read out of range\n");
			return -1;
		}
		ok = read_range(pfet, addr + cursor, mem + cursor, rlen);
		if (!ok) {
			return -1;
		}
		cursor += rlen;
	}

	return 0;
}

static bool write_range(struct pfet *pfet, address_t addr, const uint8_t *mem, address_t len, const struct chipinfo_memory *meminfo)
{
	int status;
	bool ok;

	status = do_command(pfet, "BUF:UPLOAD_BIN 0 %" PRIu32 "\r\n", len);
	if (status >= 400) {
		return false;
	}

	ok = pfet->tran->ops->send(pfet->tran, mem, len) >= 0;
	if (!ok) {
		printc_err("picofet: I/O error during upload\n");
		return false;
	}

	if (!recv_status(pfet, NULL)) {
		return false;
	}

	switch (meminfo->type) {
	case CHIPINFO_MEMTYPE_RAM:
		status = do_command(pfet, "RAM:WRITE 0 0x%"PRIx32" %"PRIu32"\r\n", addr, len);
		break;

	case CHIPINFO_MEMTYPE_FLASH:
		status = do_command(pfet, "FLASH:WRITE 0 0x%"PRIx32" %"PRIu32"\r\n", addr, len);
		break;

	default:
		printc_err("picofet: Attempting to write to memory range that isn't RAM or FLASH.\n");
		return false;
	}
	
	if (status >= 400) {
		return false;
	}

	return true;
}

static int pfet_writemem(device_t dev, address_t addr, const uint8_t *mem, address_t len)
{
	struct pfet *pfet = (struct pfet *)dev;
	const struct chipinfo_memory *m;
	address_t cursor = 0, rlen;
	bool ok;

	while (cursor < len) {
		rlen = check_range(dev->chip, addr + cursor, len - cursor, &m);
		if (m == NULL) {
			printc_err("picofet: memory write out of range\n");
			return -1;
		}
		ok = write_range(pfet, addr + cursor, mem + cursor, rlen, m);
		if (!ok) {
			return -1;
		}
		cursor += rlen;
	}

	return 0;
}

static int pfet_erase(device_t dev, device_erase_type_t type, address_t address)
{
	struct pfet *pfet = (struct pfet *)dev;
	int status = STATUS_OK;

	switch (type) {
	case DEVICE_ERASE_ALL:
		status = do_command(pfet, "FLASH:ERASE_ALL\r\n");
		break;
	case DEVICE_ERASE_MAIN:
		status = do_command(pfet, "FLASH:ERASE_MAIN\r\n");
		break;
	case DEVICE_ERASE_SEGMENT:
		status = do_command(pfet, "FLASH:ERASE_SEG 0x%"PRIx32"\r\n");
		break;
	default:
		return -1;
	}

	if (status >= 400) {
		return -1;
	}

	return 0;
}

static int pfet_getregs(device_t dev, address_t *regs)
{
	struct pfet *pfet = (struct pfet *)dev;
	int status;
	bool ok;

	memset(regs, 0, DEVICE_NUM_REGS * sizeof (*regs));
	for (int r = 0; r < DEVICE_NUM_REGS; r++) {
		status = do_command(pfet, "REG:READ %d\r\n", r);
		if (status >= 400) {
			return -1;
		}
		ok = recv_address(pfet, &regs[r]);
		if (!ok) {
			return -1;
		}
	}

	return 0;
}

static int pfet_setregs(device_t dev, const address_t *regs)
{
	struct pfet *pfet = (struct pfet *)dev;
	int status;

	for (int r = 0; r < DEVICE_NUM_REGS; r++) {
		status = do_command(pfet, "REG:WRITE %d 0x%"PRIx32"\r\n", r, regs[r]);
		if (status >= 400) {
			return -1;
		}
	}

	return 0;
}

static int pfet_ctl(device_t dev, device_ctl_t op)
{
	struct pfet *pfet = (struct pfet *)dev;
	int status;

	switch (op) {
	case DEVICE_CTL_RESET:
		status = do_command(pfet, "MCU:RESET\r\n");
		break;

	case DEVICE_CTL_RUN:
		// TODO transfer changed breakpoints to device
		status = do_command(pfet, "MCU:CONTINUE\r\n");
		break;

	case DEVICE_CTL_HALT:
		status = do_command(pfet, "MCU:HALT\r\n");
		break;

	case DEVICE_CTL_STEP:
		status = do_command(pfet, "MCU:STEP\r\n");
		break;

	default:
		printc_err("picofet: unsupported operation\n");
		return -1;
	}
	if (status >= 400) {
		return -1;
	}

	return 0;
}

static device_status_t pfet_poll(device_t dev)
{
	struct pfet *pfet = (struct pfet *)dev;
	char *lf;
	int status;

	if (delay_ms(100) < 0 || ctrlc_check()) {
		return DEVICE_STATUS_INTR;
	}

	status = do_command(pfet, "MCU:IS_HALTED\r\n");
	if (status >= 400) {
		return DEVICE_STATUS_ERROR;
	}
	lf = wait_for_line(pfet);
	unsigned long attached = strtoul(pfet->buffer, NULL, 0);
	discard_line(pfet, lf);

	if (attached) {
		return DEVICE_STATUS_HALTED;
	} else {
		return DEVICE_STATUS_RUNNING;
	}
}

static int pfet_getconfigfuses(device_t dev)
{
	struct pfet *pfet = (struct pfet *)dev;
	address_t fuses;
	char *lf;
	int status;

	status = do_command(pfet, "FUSES:READ\r\n");
	if (status >= 400) {
		return -1;
	}
	lf = wait_for_line(pfet);
	fuses = strtoul(pfet->buffer, NULL, 0);
	discard_line(pfet, lf);

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
