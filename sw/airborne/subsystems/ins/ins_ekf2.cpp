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

/**
 * @file subsystems/ins/ins_ekf2.cpp
 *
 * INS based in the EKF2 of PX4
 *
 */

#include "subsystems/ins/ins_ekf2.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"

#if defined SITL && USE_NPS
//#include "nps_fdm.h"
#include "nps_autopilot.h"
#include <stdio.h>
#endif

/** default sonar to use in INS */
#ifndef INS_EKF2_SONAR_ID
#define INS_EKF2_SONAR_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_SONAR_ID)

/** default barometer to use in INS */
#ifndef INS_EKF2_BARO_ID
#if USE_BARO_BOARD
#define INS_EKF2_BARO_ID BARO_BOARD_SENDER_ID
#else
#define INS_EKF2_BARO_ID ABI_BROADCAST
#endif
#endif
PRINT_CONFIG_VAR(INS_EKF2_BARO_ID)

/* default Gyro to use in INS */
#ifndef INS_EKF2_GYRO_ID
#define INS_EKF2_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_GYRO_ID)

/* default Accelerometer to use in INS */
#ifndef INS_EKF2_ACCEL_ID
#define INS_EKF2_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_ACCEL_ID)

/* default Magnetometer to use in INS */
#ifndef INS_EKF2_MAG_ID
#define INS_EKF2_MAG_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_MAG_ID)

/* default Alignered (lowpassed data) to use in INS */
#ifndef INS_EKF2_ALIGNER_ID
#define INS_EKF2_ALIGNER_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_ALIGNER_ID)

/* default GPS to use in INS */
#ifndef INS_EKF2_GPS_ID
#define INS_EKF2_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_EKF2_GPS_ID)

/* default opticflow velocity measurement to use in INS */
#ifndef INS_EKF2_VEL_ID
#define INS_EKF2_VEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_EKF2_VEL_ID)

static abi_event sonar_ev;
static abi_event baro_ev;
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event gps_ev;
static abi_event vel_est_ev;
static abi_event body_to_imu_ev;
static abi_event geo_mag_ev;

static void sonar_cb(uint8_t sender_id, float distance);
static void baro_cb(uint8_t sender_id, float pressure);
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void aligner_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel, struct Int32Vect3 *lp_mag);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void vel_est_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void geo_mag_cb(uint8_t sender_id, struct FloatVect3 *h);

static Ekf ekf;
parameters *ekf2_params;

/*#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z);
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_int.baro_z, &ins_int.ltp_pos.z, &ins_int.ltp_speed.z, &ins_int.ltp_accel.z);
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  if (ins_int.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_int.ltp_def.ecef.x, &ins_int.ltp_def.ecef.y, &ins_int.ltp_def.ecef.z,
                          &ins_int.ltp_def.lla.lat, &ins_int.ltp_def.lla.lon, &ins_int.ltp_def.lla.alt,
                          &ins_int.ltp_def.hmsl, &ins_int.qfe);
  }
}
#endif*/

void ins_ekf2_init(void)
{
  /* Get the ekf parameters */
  ekf2_params = ekf.getParamHandle();

/*#if USE_INS_NAV_INIT
  ins_init_origin_i_from_flightplan(&ins_int.ltp_def);
  ins_int.ltp_initialized = true;
#else
  ins_int.ltp_initialized  = false;
#endif*/


/*#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif*/

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgBARO_ABS(INS_EKF2_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgAGL(INS_EKF2_SONAR_ID, &sonar_ev, sonar_cb);
  AbiBindMsgIMU_GYRO_INT32(INS_EKF2_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_EKF2_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(INS_EKF2_ALIGNER_ID, &aligner_ev, aligner_cb);
  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgVELOCITY_ESTIMATE(INS_EKF2_VEL_ID, &vel_est_ev, vel_est_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &geo_mag_ev, geo_mag_cb);
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{

}

static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{

}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{

}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{

}

static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag)
{

}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{

}

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{

}

static void vel_est_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp,
                       float x, float y, float z,
                       float noise __attribute__((unused)))
{

}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{

}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{

}
