/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "mcu_periph/gpio.h"
#include "imu_aspirin_arch.h"
#include "subsystems/imu/imu_aspirin_2_spi.h"

void imu_aspirin_arch_init(void)
{
  /* Enable SYSCFG clock for External Interrupts on the F4 */
  //RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  rcc_periph_clock_enable(RCC_SYSCFG);

  /* Configure the interrupt pins as input */
  /*rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO14);
  gpio_set_af(GPIOC, GPIO_AF0, GPIO14);*/
  gpio_setup_input(GPIOC, GPIO14); // HMC58xx interrupt pin
  gpio_setup_input(GPIOB, GPIO5); // MPU60x0 interrupt pin

  /* Setup the external interrupt for the HMC58xx */
  exti_select_source(EXTI14, GPIOC);
  exti_set_trigger(EXTI14, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI14);

  /* Setup the external interrupt for the MPU60x0 */
  exti_select_source(EXTI5, GPIOB);
  exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI5);

  /* Enable NVIC IRQ for the HMC58xx and MPU60x0 */
  nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x0F);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

/* External interrupt for the HMC58xx */
void exti15_10_isr(void)
{
  exti_reset_request(EXTI14);
  imu_aspirin2.hmc58xx_ready = TRUE;
}

/* External interrupt for the MPU60x0 */
void exti9_5_isr(void)
{
  exti_reset_request(EXTI5);
  imu_aspirin2.mpu60x0_ready = TRUE;
}
