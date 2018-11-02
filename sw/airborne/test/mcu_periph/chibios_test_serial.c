/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
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

/* ChibiOS includes */
#include "ch.h"

/* paparazzi includes */
#include "mcu.h"
#include "led.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/uart_arch.h"

/* This is a copy of the struct from the uart_arch.c file
 * there is no other way to access the semaphor and wait
 * far rx data to come in. There is no function for that.
 */
struct SerialInit {
  SerialConfig *conf;
  semaphore_t *rx_sem;
  semaphore_t *tx_sem;
  mutex_t *rx_mtx;
  mutex_t *tx_mtx;
};

/*
 * Thread Area Definitions
 */
#define CH_CFG_THREAD_AREA_MAIN_PERIODIC 128

/*
 * Thread Area Initialization
 */
static THD_WORKING_AREA(wa_thd_main_periodic_05, CH_CFG_THREAD_AREA_MAIN_PERIODIC);
static THD_WORKING_AREA(wa_thd_rx, CH_CFG_THREAD_AREA_MAIN_PERIODIC);

/*
 * Static Thread Definitions
 */
static __attribute__((noreturn)) void thd_main_periodic_05(void *arg);
static __attribute__((noreturn)) void thd_rx(void *arg);

/*
 * Test Thread
 *
 * Replaces main_periodic_05()
 *
 */
static __attribute__((noreturn)) void thd_main_periodic_05(void *arg)
{
  chRegSetThreadName("thd_blinker");
  (void) arg;
  systime_t time = chVTGetSystemTime();
  while (TRUE)
  {
    time += TIME_MS2I(500);
#ifdef SYS_TIME_LED
      LED_TOGGLE(SYS_TIME_LED);
#endif
    chThdSleepUntil(time);
  }
}

/*
 * Serial RX thread
 */
__attribute__((noreturn)) void thd_rx(void *arg)
{
  chRegSetThreadName("rx_thread");
  (void) arg;

  uint8_t charbuf;
  while (TRUE) {
#ifdef LED_GREEN
      LED_TOGGLE(LED_GREEN);
#endif
    chSemWait(((struct SerialInit *)SERIAL_PORT.init_struct)->rx_sem);
    charbuf = uart_getch(&SERIAL_PORT);
    if (charbuf != 0) {
    	uart_put_byte(&SERIAL_PORT, 0, charbuf);
    }
  }
}

int main(void)
{
  mcu_init();

  /*
   * Init threads
   */
  chThdCreateStatic(wa_thd_main_periodic_05, sizeof(wa_thd_main_periodic_05), NORMALPRIO, thd_main_periodic_05, NULL);
  chThdCreateStatic(wa_thd_rx, sizeof(wa_thd_rx), NORMALPRIO, thd_rx, NULL);


  while (1) {
    /* sleep for 1s */
    sys_time_ssleep(1);
    uart_put_byte(&SERIAL_PORT, 0, 'N');
    uart_put_byte(&SERIAL_PORT, 0, 'i');
    uart_put_byte(&SERIAL_PORT, 0, 'c');
    uart_put_byte(&SERIAL_PORT, 0, 'e');

    sys_time_msleep(500);
    uint8_t tx_switch[] = " work!\r\n";
    uart_put_buffer(&SERIAL_PORT, 0, tx_switch, sizeof(tx_switch));
  }

  return 0;
}
