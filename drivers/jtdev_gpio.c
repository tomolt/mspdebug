/* MSPDebug - gpio device interface
 *  Copyright (C) 2014 TTI GmbH - TGU Smartmote
 *  Author(s): Jan Willeke (willeke@smartmote.de)
 *
 * Linux /sys/class/gpio interface to msp430 jtag
 * inspired by urjtag and jtdev.c
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

#include <stdint.h>
#include "jtaglib.h"
#include "output.h"

#if defined(__linux__)
/*===== includes =============================================================*/

#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include <gpiod.h>

/* pin mapping */
enum {
    GPIO_TDI = 0,
    GPIO_TCK,
    GPIO_TMS,
    GPIO_TDO,
    GPIO_RST,
    GPIO_TST,
    GPIO_REQUIRED
};

static unsigned int               jtag_gpios[6];
static struct gpiod_chip         *chip;
static struct gpiod_line_request *line_req;

static int
gpio_open ()
{
    int s;
    unsigned int i;
    struct gpiod_request_config *req_cfg;
    struct gpiod_line_config *line_cfg;

    const char *chip_path = "/dev/gpiochip0";
    chip = gpiod_chip_open(chip_path);
    if (!chip) {
        printf("cannot open gpiod chip\n");
        return -1;
    }

    req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "mspdebug");

    line_cfg = gpiod_line_config_new();

    for (i = 0; i < GPIO_REQUIRED; i++) {
        struct gpiod_line_settings *line_set;

        line_set = gpiod_line_settings_new();
        gpiod_line_settings_set_direction(line_set, (i == GPIO_TDO) ? GPIOD_LINE_DIRECTION_INPUT : GPIOD_LINE_DIRECTION_OUTPUT);

        if (i != GPIO_TDO) {
            gpiod_line_settings_set_output_value(line_set, GPIOD_LINE_VALUE_INACTIVE);
        }

        s = gpiod_line_config_add_line_settings(line_cfg, &jtag_gpios[i], 1, line_set);
        if (s < 0) {
            printf("error: gpiod_line_config_add_line_settings");
            return -1;
        }

        gpiod_line_settings_free(line_set);
    }

    line_req = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!line_req) {
            printf("error: gpiod_chip_request_lines");
            return -1;
    }

    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    return 0;
}

static int
gpio_parse_config (const char *params)
{
    struct option{
        char* name;
        int  num;
    };
    static const struct option ops[] = {
        {"tms=",GPIO_TMS},
        {"tdi=",GPIO_TDI},
        {"tdo=",GPIO_TDO},
        {"tck=",GPIO_TCK},
        {"rst=",GPIO_RST},
        {"tst=",GPIO_TST}
    };
    int i;

    for (i = 0;i < 6; i++) {
        char* help;
        help = strstr(params,ops[i].name);
        if (help)
            jtag_gpios[ops[i].num] = atoi(help+4);
        else
            return -1;
        printf("gpio %s %d\n", ops[i].name,jtag_gpios[ops[i].num]);
    }
    return 0;
}


static int jtgpio_open(struct jtdev *p, const char *device)
{
    if (gpio_parse_config(device)) {
        printf("gpio: failed parsing parameters\n");
        return -1;
    }
    return gpio_open();
}

static void jtgpio_close(struct jtdev *p)
{
    printf("JTAG_CLOSE\n");

    gpiod_line_request_release(line_req);
    gpiod_chip_close(chip);
}

static void jtgpio_power_on(struct jtdev *p)
{
    printf("JTAG_power on\n");
}

static void jtgpio_power_off(struct jtdev *p)
{
    printf("JTAG_power off\n");
}

static void jtgpio_connect(struct jtdev *p)
{
    printf("JTAG_connect \n");
}

static void jtgpio_release(struct jtdev *p)
{
    printf("JTAG_release\n");
}

static void jtgpio_tck(struct jtdev *p, int out)
{
    gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TCK], out ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static void jtgpio_tms(struct jtdev *p, int out)
{
    gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TMS], out ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static void jtgpio_tdi(struct jtdev *p, int out)
{
    gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TDI], out ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static void jtgpio_rst(struct jtdev *p, int out)
{
    gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_RST], out ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static void jtgpio_tst(struct jtdev *p, int out)
{
    gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TST], out ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static int jtgpio_tdo_get(struct jtdev *p)
{
    enum gpiod_line_value line_val = gpiod_line_request_get_value(line_req, jtag_gpios[GPIO_TDO]);
    return line_val == GPIOD_LINE_VALUE_ACTIVE;
}

static void jtgpio_tclk(struct jtdev *p, int out)
{
    gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TDI], out ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

static int jtgpio_tclk_get(struct jtdev *p)
{
    enum gpiod_line_value line_val = gpiod_line_request_get_value(line_req, jtag_gpios[GPIO_TDI]);
    return line_val == GPIOD_LINE_VALUE_ACTIVE;
}

static void jtgpio_tclk_strobe(struct jtdev *p, unsigned int count)
{
    int i;
    for (i = 0; i < count; i++) {
        gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TDI], GPIOD_LINE_VALUE_ACTIVE);
        gpiod_line_request_set_value(line_req, jtag_gpios[GPIO_TDI], GPIOD_LINE_VALUE_INACTIVE);
    }
}

static void jtgpio_led_green(struct jtdev *p, int out)
{
    printf("led green\n");
}

static void jtgpio_led_red(struct jtdev *p, int out)
{
    printf("led red\n");
}


#else /* __linux__ */


static int jtgpio_open(struct jtdev *p, const char *device)
{
    printc_err("jtdev: driver is not supported on this platform\n");
    p->failed = 1;
    return -1;
}

static void jtgpio_close(struct jtdev *p) { }

static void jtgpio_power_on(struct jtdev *p) { }
static void jtgpio_power_off(struct jtdev *p) { }
static void jtgpio_connect(struct jtdev *p) { }
static void jtgpio_release(struct jtdev *p) { }

static void jtgpio_tck(struct jtdev *p, int out) { }
static void jtgpio_tms(struct jtdev *p, int out) { }
static void jtgpio_tdi(struct jtdev *p, int out) { }
static void jtgpio_rst(struct jtdev *p, int out) { }
static void jtgpio_tst(struct jtdev *p, int out) { }
static int jtgpio_tdo_get(struct jtdev *p) { return 0; }

static void jtgpio_tclk(struct jtdev *p, int out) { }
static int jtgpio_tclk_get(struct jtdev *p) { return 0; }
static void jtgpio_tclk_strobe(struct jtdev *p, unsigned int count) { }

static void jtgpio_led_green(struct jtdev *p, int out) { }
static void jtgpio_led_red(struct jtdev *p, int out) { }
#endif


const struct jtdev_func jtdev_func_gpio = {
    .jtdev_open        = jtgpio_open,
    .jtdev_close       = jtgpio_close,
    .jtdev_power_on    = jtgpio_power_on,
    .jtdev_power_off   = jtgpio_power_off,
    .jtdev_connect     = jtgpio_connect,
    .jtdev_release     = jtgpio_release,
    .jtdev_tck	     = jtgpio_tck,
    .jtdev_tms	     = jtgpio_tms,
    .jtdev_tdi	     = jtgpio_tdi,
    .jtdev_rst	     = jtgpio_rst,
    .jtdev_tst	     = jtgpio_tst,
    .jtdev_tdo_get     = jtgpio_tdo_get,
    .jtdev_tclk	     = jtgpio_tclk,
    .jtdev_tclk_get    = jtgpio_tclk_get,
    .jtdev_tclk_strobe = jtgpio_tclk_strobe,
    .jtdev_led_green   = jtgpio_led_green,
    .jtdev_led_red     = jtgpio_led_red,

    .jtdev_ir_shift    = jtag_default_ir_shift,
    .jtdev_dr_shift_8  = jtag_default_dr_shift_8,
    .jtdev_dr_shift_16 = jtag_default_dr_shift_16,
    .jtdev_tms_sequence= jtag_default_tms_sequence,
    .jtdev_init_dap    = jtag_default_init_dap

};
