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

#include <stdlib.h>
#include <string.h>

#include "picofet.h"
#include "picofet_proto.h"

#include "cdc_acm.h"

#include "util.h"
#include "output.h"

struct pfet {
	struct device device;
};

/* Returns 1 on success, 0 on failure.
 */
static int init_pfet(struct pfet *pfet)
{
	(void)pfet;
	return 1;
}

static void deinit_pfet(struct pfet *pfet)
{
	(void)pfet;
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

	if (!init_pfet(pfet)) {
		tran->ops->destroy(tran);
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
