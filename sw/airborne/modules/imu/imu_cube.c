/*
 * Copyright (C) 2022 Freek van tieen <freek.v.tienen@gmail.com>
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
 * @file modules/imu/imu_cube.c
 * Driver for the IMU's in the Cube autopilots.
 */

#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/spi.h"
#include "peripherals/invensense2.h"


struct invensense2_t imu1;

void imu_cube_init(void)
{
  imu1.abi_id = IMU_CUBE1_ID;
  imu1.bus = INVENSENSE2_SPI;
  imu1.spi.p = &CUBE_IMU1_SPI_DEV;
  imu1.spi.slave_idx = CUBE_IMU1_SPI_SLAVE_IDX;
  imu1.gyro_dlpf = INVENSENSE2_GYRO_DLPF_377HZ;
  imu1.gyro_range = INVENSENSE2_GYRO_RANGE_4000DPS;
  imu1.accel_dlpf = INVENSENSE2_ACCEL_DLPF_499HZ;
  imu1.accel_range = INVENSENSE2_ACCEL_RANGE_30G;
  invensense2_init(&imu1);
}

void imu_cube_periodic(void)
{
  invensense2_periodic(&imu1);
}

void imu_cube_event(void)
{
  invensense2_event(&imu1);
}
