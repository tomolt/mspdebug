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

#ifndef PICOFET_PROTO_H_
#define PICOFET_PROTO_H_

#define PFET_USB_BAUD_RATE 460800

/* Not finalized yet */

#define STATUS_OK                200
#define STATUS_UNKNOWN_COMMAND   500
#define STATUS_INVALID_ARGUMENTS 501
#define STATUS_INTEGER_OVERFLOW  502
#define STATUS_COMMAND_TOO_LONG  503
#define STATUS_FUSE_BLOWN        550
#define STATUS_INVALID_JTAG_ID   551
#define STATUS_PUC_FAILED        552
#define STATUS_TOO_MANY_BREAKS   553
#define STATUS_OUT_OF_BOUNDS     554

#define STATUS_CONTENT_MISMATCH  201
#define STATUS_CONTINUE_TRANSFER 350
#define STATUS_TIMED_OUT         400
#define STATUS_TRANSFER_FAILED   401
#define STATUS_PROGRAMMER_FROZE  402

#endif
