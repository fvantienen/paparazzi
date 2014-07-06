/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi

 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "modules/debug/debug_gpio.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/sys_time_arch.h"

#if !defined(DEBUG_GPIO_SEL)
#error "You must define a GPIO output for the DEBUG_GPIO_SEL"
#endif

#if !defined(DEBUG_GPIO_DATA)
#error "You must define a GPIO output for the DEBUG_GPIO_DATA"
#endif

uint32_t last_module = 0;
bool debug_gpio_initialized = FALSE;
static void debug_gpio_send(uint32_t value);

void debug_gpio_init(void) {
	// Check if already done
	if(debug_gpio_initialized) {
		return;
	}

	// Set both the select and the data pin to output
	gpio_setup_output(DEBUG_GPIO_SEL);
	gpio_setup_output(DEBUG_GPIO_DATA);

	// Set to init position
	gpio_clear(DEBUG_GPIO_SEL);
	gpio_clear(DEBUG_GPIO_DATA);

	// Send two pulses on the selection pin to identify it
	sys_time_usleep(1000);
	gpio_set(DEBUG_GPIO_SEL);
	sys_time_usleep(100);
	gpio_clear(DEBUG_GPIO_SEL);
	sys_time_usleep(100);
	gpio_set(DEBUG_GPIO_SEL);
	sys_time_usleep(100);
	gpio_clear(DEBUG_GPIO_SEL);
	sys_time_usleep(100);

	debug_gpio_initialized = TRUE;
}

void debug_gpio(__attribute__((unused)) uint32_t module, uint32_t value) {
	// Initialize if needed
	if(!debug_gpio_initialized) {
		debug_gpio_init();
	}

	// Send the module if needed
	/*if(last_module != module) {
		gpio_set(DEBUG_GPIO_DATA);
		gpio_clear(DEBUG_GPIO_DATA);

		debug_gpio_send(module);
	}*/

	// Send the value
	debug_gpio_send(value);
}

static void debug_gpio_send(uint32_t value) {
	if(value > 30)
		return;

	gpio_set(DEBUG_GPIO_SEL);

	// Send a pulse for every value
	for(uint32_t i = 0; i < value; i++) {
		gpio_set(DEBUG_GPIO_DATA);
		gpio_clear(DEBUG_GPIO_DATA);
		
	}
	gpio_clear(DEBUG_GPIO_SEL);
}
