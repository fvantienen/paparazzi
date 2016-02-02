/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *               2016 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
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
 */

/*#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>*/

#include "mcu_periph/gpio.h"

bool_t hmc58xx_has_data = FALSE;

void hmc58xx_arch_init(void)
{
  /* Configure the interrupt pin as input */
  gpio_setup_input(RCC_GPIOB, GPIO5);

  /* Setup the external interrupt */
  exti_select_source(EXTI5, GPIOB);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI5);

  /* Enable NVIC IRQ */
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0f);
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

void exti9_5_isr(void)
{
  exti_reset_request(EXTI5);
  hmc58xx_has_data = TRUE;
}
