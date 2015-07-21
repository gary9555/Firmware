/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file servo_ctl.c
 *
 * gripper servo via GPIO driver.
 *
 * @author Vinh Nguyen
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_pwm_output.h>
#include <modules/px4iofirmware/protocol.h>

struct gpio_servo_s {
	struct work_s work;
	int gpio_fd;
	bool use_io;
	int pin;
	bool servo_state;
	int counter;
};

static struct gpio_servo_s *gpio_servo_data;
static bool gpio_servo_started = false;

__EXPORT int gpio_servo_main(int argc, char *argv[]);

void gpio_servo_start(FAR void *arg);

int gpio_servo_main(int argc, char *argv[])
{
	if (argc < 2) {

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
		errx(1, "usage: gpio_servo {start|stop} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT pin number (default: 1)"
		    );
#endif

	} else {

		if (!strcmp(argv[1], "start")) {
			if (gpio_servo_started) {
				errx(1, "already running");
			}

			bool use_io = false;

			/* by default use GPIO_EXT_1 on FMUv1 and GPIO_SERVO_1 on FMUv2 */
			int pin = 1;

			/* pin name to display */

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
			char pin_name[] = "AUX OUT 1";
#endif

			if (argc > 2) {
				if (!strcmp(argv[2], "-p")) {

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
					unsigned int n = strtoul(argv[3], NULL, 10);

					if (n >= 1 && n <= 6) {
						use_io = false;
						pin = 1 << (n - 1);
						snprintf(pin_name, sizeof(pin_name), "AUX OUT %d", n);

					} else {
						errx(1, "unsupported pin: %s", argv[3]);
					}

#endif
				}
			}

			gpio_servo_data = malloc(sizeof(struct gpio_servo_s));
			memset(gpio_servo_data, 0, sizeof(struct gpio_servo_s));
			gpio_servo_data->use_io = use_io;
			gpio_servo_data->pin = pin;
			int ret = work_queue(LPWORK, &(gpio_servo_data->work), gpio_servo_start, gpio_servo_data, 0);

			if (ret != 0) {
				errx(1, "failed to queue work: %d", ret);

			} else {
				gpio_servo_started = true;
				warnx("start, using pin: %s", pin_name);
				exit(0);
			}

		} else if (!strcmp(argv[1], "stop")) {
			if (gpio_servo_started) {
				gpio_servo_started = false;
				warnx("stop");
				exit(0);

			} else {
				errx(1, "not running");
			}

		} else {
			errx(1, "unrecognized command '%s', only supporting 'start' or 'stop'", argv[1]);
		}
	}
}

//takes a pointer to another gpio_servo_s struct
void gpio_servo_start(FAR void *arg)
{
	FAR struct gpio_servo_s *priv = (FAR struct gpio_servo_s *)arg;

	char *gpio_dev;

	// sets all actions to the FMU/AUX pins
	gpio_dev = PX4FMU_DEVICE_PATH;

	// open GPIO device
	priv->gpio_fd = open(gpio_dev, 0);

	if (priv->gpio_fd < 0) {
		// TODO find way to print errors
		//printf("gpio_servo: GPIO device \"%s\" open fail\n", gpio_dev);
		gpio_servo_started = false;
		return;
	}

	/* configure GPIO pin */
	/* px4fmu only, px4io doesn't support GPIO_SET_OUTPUT and will ignore */
	ioctl(priv->gpio_fd, GPIO_SET_OUTPUT, priv->pin);

	// sets PWM values
	ret = ioctl(priv->gpio_fd, PWM_SERVO_SET_ARM_OK, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_SET_ARM_OK");
	}

	/* tell IO that the system is armed (it will output values if safety is off) */
	ret = ioctl(priv->gpio_fd, PWM_SERVO_ARM, 0);

	if (ret != OK) {
		err(1, "PWM_SERVO_ARM");
	}

	/* add worker to queue */
	int ret = work_queue(LPWORK, &priv->work, gpio_servo_cycle, priv, 0);

	if (ret != 0) {
		// TODO find way to print errors
		//printf("gpio_servo: faiservo to queue work: %d\n", ret);
		gpio_servo_started = false;
		return;
	}
}

void gpio_servo_stop(FAR void *arg)
{
	FAR struct gpio_servo_s *priv = (FAR struct gpio_servo_s *)arg;
	ioctl(priv->gpio_fd, GPIO_CLEAR, priv->pin);
}