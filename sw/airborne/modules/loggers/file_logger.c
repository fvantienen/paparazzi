/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "subsystems/imu.h"
#include "subsystems/actuators/motor_mixing.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_indi.h"
#include "state.h"
#include "boards/bebop/actuators.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH "/data/video/usb/"
#endif

/** The file pointer */
static FILE* file_logger;

/** Start the file logger and open a new file */
void file_logger_start(void) {
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz,rpm_lf,rpm_rf,rpm_rb,rpm_lb\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void) {
  fclose(file_logger);
  file_logger = NULL;
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat* quat = stateGetNedToBodyQuat_i();

  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf\n",
    counter,
    imu.gyro.p,
    imu.gyro.q,
    imu.gyro.r,
    imu.accel.x,
    imu.accel.y,
    imu.accel.z,
    imu.mag.x,
    imu.mag.y,
    imu.mag.z,
    stabilization_cmd[COMMAND_THRUST],
    stabilization_cmd[COMMAND_ROLL],
    stabilization_cmd[COMMAND_PITCH],
    stabilization_cmd[COMMAND_YAW],
    quat->qi,
    quat->qx,
    quat->qy,
    quat->qz,
    actuators_bebop.rpm_obs[0],
    actuators_bebop.rpm_obs[1],
    actuators_bebop.rpm_obs[2],
    actuators_bebop.rpm_obs[3],
    indi_u.p,
    indi_du.p,
    u_in.p,
    indi_u.q,
    indi_u.r
  );
  counter++;
}
