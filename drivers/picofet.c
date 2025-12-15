/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2025 Thomas Oltmann
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

#include "picofet.h"
#include "picofet_proto.h"

#include "cdc_acm.h"

#include "util.h"
#include "output.h"

#define BUFFER_SIZE 256

struct pfet {
	struct device device;
	transport_t   tran;
	char         *buffer;
	unsigned      buffered;
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

static bool recv_status(struct pfet *pfet, int *status)
{
	char *lf;

	lf = wait_for_line(pfet);
	if (!lf) {
		return false;
	}
	if (status) {
		*status = atoi(pfet->buffer);
	}
	discard_line(pfet, lf);

	return true;
}

static bool recv_address(struct pfet *pfet, address_t *address)
{
	char *lf, *end;

	lf = wait_for_line(pfet);
	if (!lf) {
		return false;
	}
	if (address) {
		*address = strtoul(pfet->buffer, &end, 0);
		if (pfet->buffer[0] == '\0' || *end != '\0') {
			return false;
		}
	}
	discard_line(pfet, lf);

	return true;
}

static int do_command(struct pfet *pfet, const char *format, ...)
{
	va_list va;
	int len, status;
	bool ok;

	va_start(va, format);
	len = vsprintf(pfet->buffer, format, va);
	if (len < 0) {
		return -1;
	}
	va_end(va);

	ok = pfet->tran->ops->send(pfet->tran, (uint8_t *)pfet->buffer, len) >= 0;
	if (!ok) {
		return -1;
	}

	ok = recv_status(pfet, &status);
	if (!ok) {
		return -1;
	}

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

#if 0
	address_t capacity;
	bool ok;
	status = do_command(pfet, "BUF:CAPACITY\r\n");
	if (status != STATUS_OK) {
		printc_err("picofet: %03d\n", status);
		return false;
	}

	ok = recv_address(pfet, &capacity);
	if (!ok) {
		return false;
	}
#endif

	status = do_command(pfet, "MCU:ATTACH\r\n");
	if (status != STATUS_OK) {
		printc_err("picofet: %03d\n", status);
		return false;
	}

	return true;
}

static void deinit_pfet(struct pfet *pfet)
{
	int status;
	
	status = do_command(pfet, "MCU:DETACH\r\n");
	if (status != STATUS_OK) {
		printc_err("picofet: %03d\n", status);
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

	tran = cdc_acm_open(args->path, args->requested_serial, PFET_USB_BAUD_RATE, 0x2E8A, 0x0009);
	if (!tran) {
		tran = cdc_acm_open(args->path, args->requested_serial, PFET_USB_BAUD_RATE, 0x2E8A, 0x000A);
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

static int pfet_readmem(device_t dev, address_t addr, uint8_t *mem, address_t len)
{
	return 0;
}

static int pfet_writemem(device_t dev, address_t addr, const uint8_t *mem, address_t len)
{
	return 0;
}

static int pfet_erase(device_t dev, device_erase_type_t type, address_t address)
{
	return 0;
}

static int pfet_getregs(device_t dev, address_t *regs)
{
	return 0;
}

static int pfet_setregs(device_t dev, const address_t *regs)
{
	return 0;
}

static int pfet_ctl(device_t dev, device_ctl_t op)
{
	return 0;
}

static device_status_t pfet_poll(device_t dev)
{
	return DEVICE_STATUS_ERROR;
}

static int pfet_getconfigfuses(device_t dev)
{
	return 0;
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
