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
 * @file modules/sensors/mag_lis3mdl.h
 *
 * Module wrapper for ST LIS3MDL magnetometer.
 */

#ifndef MAG_LIS3MDL_H
#define MAG_LIS3MDL_H

#include "peripherals/lis3mdl_i2c.h"

extern struct lis3mdl_t mag_lis3mdl;

extern void mag_lis3mdl_module_init(void);
extern void mag_lis3mdl_module_periodic(void);
extern void mag_lis3mdl_module_event(void);
extern void mag_lis3mdl_report(void);

#endif // MAG_LIS3MDL_H
