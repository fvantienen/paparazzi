/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file peripherals/lis3mdl.h
 *
 * Driver for ST LIS3MDL 3D accelerometer and magnetometer.
 */
#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "std.h"
/* Address and register definitions */
#include "peripherals/lis3mdl_regs.h"

/* LIS3MDL default conf */
#ifndef LIS3MDL_DEFAULT_MODR
#define LIS3MDL_DEFAULT_MODR (LIS3MDL_CTRL_REG1_DO0 | LIS3MDL_CTRL_REG1_DO1 | LIS3MDL_CTRL_REG1_DO2) // Magneto Data Output Rate (80Hz)
#endif

#ifndef LIS3MDL_DEFAULT_MFS
#define LIS3MDL_DEFAULT_MFS 0 // Magneto gain configuration (+/- 4 Gauss)
#endif

#ifndef LIS3MDL_DEFAULT_MD
#define LIS3MDL_DEFAULT_MD 0 // Magneto continious conversion mode
#endif

struct Lis3mdlConfig {
  uint8_t rate;  ///< Data Output Rate Bits (Hz)
  uint8_t scale;  ///< Full scale gain configuration (Gauss)
  uint8_t mode;  ///< Measurement mode
};

/** status states */
enum Lis3mdlStatus {
  LIS_STATE_UNINIT,
  LIS_STATE_RESET,
  LIS_STATE_CTRL_REG1,
  LIS_STATE_CTRL_REG2,
  LIS_STATE_CTRL_REG3,
  LIS_STATE_IDLE,
  LIS_STATE_READ
};

static inline void lis3mdl_mag_set_default_config(struct Lis3mdlConfig *c)
{
  c->rate = LIS3MDL_DEFAULT_MODR;
  c->scale = LIS3MDL_DEFAULT_MFS;
  c->mode = LIS3MDL_DEFAULT_MD;
}
#endif // LIS3MDL_H
