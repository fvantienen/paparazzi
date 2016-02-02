/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/debug/timing_debug.h"
 * @author Freek van Tienen
 * This module debugs the timings of Paparazzi using GPIOs as output.
 */

#ifndef TIMING_DEBUG_H
#define TIMING_DEBUG_H

#include "led.h"

#if USE_TIMING_DEBUG
#define TD_ON(i) LED_ON(D ## i)
#define TD_OFF(i) LED_OFF(D ## i)
#define TD_TOGGLE(i) LED_TOGGLE(D ## i)

extern void timing_debug_init(void);

#else
#define TD_ON(i) {}
#define TD_OFF(i) {}
#define TD_TOGGLE(i) {}
#endif

#endif

