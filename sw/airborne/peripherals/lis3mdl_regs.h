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
 * @file peripherals/lis3mdl_regs.h
 * Register defs for ST LIS3MDL 3D magnetometer.
 */

#ifndef LIS3MDL_REGS_H
#define LIS3MDL_REGS_H

/* default I2C address */
#define LIS3MDL_ADDR 0x3C

/* Registers */
#define  LIS3MDL_AD_WHO_AM_I                0x0F
#define  LIS3MDL_AD_CTRL_REG1               0x20
#define  LIS3MDL_AD_CTRL_REG2               0x21
#define  LIS3MDL_AD_CTRL_REG3               0x22
#define  LIS3MDL_AD_CTRL_REG4               0x23
#define  LIS3MDL_AD_CTRL_REG5               0x24
#define  LIS3MDL_AD_STATUS_REG              0x27
#define  LIS3MDL_AD_OUT_X_L                 0x28
#define  LIS3MDL_AD_OUT_X_H                 0x29
#define  LIS3MDL_AD_OUT_Y_L                 0x2A
#define  LIS3MDL_AD_OUT_Y_H                 0x2B
#define  LIS3MDL_AD_OUT_Z_L                 0x2C
#define  LIS3MDL_AD_OUT_Z_H                 0x2D
#define  LIS3MDL_AD_TEMP_OUT_L              0x2E
#define  LIS3MDL_AD_TEMP_OUT_H              0x2F
#define  LIS3MDL_AD_INT_CFG                 0x30
#define  LIS3MDL_AD_INT_SOURCE              0x31
#define  LIS3MDL_AD_INT_THS_L               0x32
#define  LIS3MDL_AD_INT_THS_H               0x33

/* LIS3MDL_AD_CTRL_REG1 register bits definitions */
#define LIS3MDL_CTRL_REG1_MASK              0xFF
#define LIS3MDL_CTRL_REG1_ST                (1 << 0)
#define LIS3MDL_CTRL_REG1_FAST_ODR          (1 << 1)
#define LIS3MDL_CTRL_REG1_DO0               (1 << 2)
#define LIS3MDL_CTRL_REG1_DO1               (1 << 3)
#define LIS3MDL_CTRL_REG1_DO2               (1 << 4)
#define LIS3MDL_CTRL_REG1_OM0               (1 << 5)
#define LIS3MDL_CTRL_REG1_OM1               (1 << 6)
#define LIS3MDL_CTRL_REG1_TEMP_EN           (1 << 7)

/* LIS3MDL_CTRL_REG2 register bits definitions */
#define LIS3MDL_CTRL_REG2_MASK              0x6C
#define LIS3MDL_CTRL_REG2_SOFT_RST          (1 << 2)
#define LIS3MDL_CTRL_REG2_REBOOT            (1 << 3)
#define LIS3MDL_CTRL_REG2_FS_MASK           0x60
#define LIS3MDL_CTRL_REG2_FS0               (1 << 5)
#define LIS3MDL_CTRL_REG2_FS1               (1 << 6)

/* LIS3MDL_CTRL_REG3 register bits definitions */
#define LIS3MDL_CTRL_REG3_MASK              0x27
#define LIS3MDL_CTRL_REG3_MD0               (1 << 0)
#define LIS3MDL_CTRL_REG3_MD1               (1 << 1)
#define LIS3MDL_CTRL_REG3_SIM               (1 << 2)
#define LIS3MDL_CTRL_REG3_LP                (1 << 5)

/* LIS3MDL_CTRL_REG4 register bits definitions */
#define LIS3MDL_CTRL_REG4_MASK              0x0E
#define LIS3MDL_CTRL_REG4_BLE               (1 << 1)
#define LIS3MDL_CTRL_REG4_OMZ0              (1 << 2)
#define LIS3MDL_CTRL_REG4_OMZ1              (1 << 3)

/* LIS3MDL_CTRL_REG5 register bits definitions */
#define LIS3MDL_CTRL_REG5_MASK              0xC0
#define LIS3MDL_CTRL_REG5_BDU               (1 << 6)
#define LIS3MDL_CTRL_REG5_FAST_READ         (1 << 7)

/* LIS3MDL_STATUS_REG register bits definitions */
#define LIS3MDL_STATUS_REG_MASK             0xFF
#define LIS3MDL_STATUS_REG_XDA              (1 << 0)
#define LIS3MDL_STATUS_REG_YDA              (1 << 1)
#define LIS3MDL_STATUS_REG_ZDA              (1 << 2)
#define LIS3MDL_STATUS_REG_XYZDA            (1 << 3)
#define LIS3MDL_STATUS_REG_XOR              (1 << 4)
#define LIS3MDL_STATUS_REG_YOR              (1 << 5)
#define LIS3MDL_STATUS_REG_ZOR              (1 << 6)
#define LIS3MDL_STATUS_REG_ZYXOR            (1 << 7)

#endif // LIS3MDL_REGS_H
