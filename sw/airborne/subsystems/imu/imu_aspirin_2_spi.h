/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/imu/imu_aspirin_2_spi.h
 * Driver for the Aspirin v2.x IMU using SPI for the MPU6000.
 */

#ifndef IMU_ASPIRIN_2_SPI_H
#define IMU_ASPIRIN_2_SPI_H

#include "std.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"

/* default sensitivitiy */
#include "subsystems/imu/imu_mpu60x0_defaults.h"
#include "peripherals/mpu60x0_spi.h"

struct ImuAspirin2Spi {
  struct Mpu60x0_Spi mpu;                   ///< Main MPU SPI structure

  struct spi_transaction wait_slave4_trans; ///< Busy waiting slave 4 spi to i2c transaction
  volatile uint8_t wait_slave4_tx_buf[1];   ///< Slave 4 i2c transmit buffer
  volatile uint8_t wait_slave4_rx_buf[2];   ///< Slave 4 i2c receive buffer
  volatile bool_t slave4_ready;             ///< Is set to TRUE when slave 4 i2c transaction is done

  volatile bool_t hmc58xx_ready;            ///< Is set to TRUE when we had an interrupt from the HMC
  volatile bool_t mpu60x0_ready;            ///< Is set to TRUE when we had an interrupt from the MPU
};

extern struct ImuAspirin2Spi imu_aspirin2;

extern void imu_aspirin2_event(void);

#define ImuEvent imu_aspirin2_event

#endif /* IMU_ASPIRIN_2_H */
