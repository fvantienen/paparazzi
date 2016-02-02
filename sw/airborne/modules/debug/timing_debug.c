/*
 * Copyright (C) Freek van Tienen
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/debug/timing_debug.c"
 * @author Freek van Tienen
 * This module debugs the timings of Paparazzi using GPIOs as output.
 */

#include "modules/debug/timing_debug.h"

void timing_debug_init(void) {
#ifdef LED_D0_GPIO
  LED_INIT(D0);
  LED_OFF(D0);
#endif

#ifdef LED_D1_GPIO
  LED_INIT(D1);
  LED_OFF(D1);
#endif

#ifdef LED_D2_GPIO
  LED_INIT(D2);
  LED_OFF(D2);
#endif

#ifdef LED_D3_GPIO
  LED_INIT(D3);
  LED_OFF(D3);
#endif

#ifdef LED_D4_GPIO
  LED_INIT(D4);
  LED_OFF(D4);
#endif

#ifdef LED_D5_GPIO
  LED_INIT(D5);
  LED_OFF(D5);
#endif

#ifdef LED_D6_GPIO
  LED_INIT(D6);
  LED_OFF(D6);
#endif

#ifdef LED_D7_GPIO
  LED_INIT(D7);
  LED_OFF(D7);
#endif

#ifdef LED_D8_GPIO
  LED_INIT(D8);
  LED_OFF(D8);
#endif

#ifdef LED_D9_GPIO
  LED_INIT(D9);
  LED_OFF(D9);
#endif

#ifdef LED_D10_GPIO
  LED_INIT(D10);
  LED_OFF(D10);
#endif

#ifdef LED_D11_GPIO
  LED_INIT(D11);
  LED_OFF(D11);
#endif
}
