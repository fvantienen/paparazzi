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
#include "EKF/ekf.h"

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

struct ekf2_t {
    uint32_t gyro_stamp;
    uint32_t gyro_dt;
    uint32_t accel_stamp;
    uint32_t accel_dt;
    FloatRates gyro;
    FloatVect3 accel;
    bool gyro_valid;
    bool accel_valid;

    struct OrientationReps body_to_imu;
};
static struct ekf2_t ekf2;

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
  ekf2_params->mag_delay_ms = 0;
  ekf2_params->baro_delay_ms = 0;
  ekf2_params->gps_delay_ms = 200;
  ekf2_params->flow_delay_ms = 5;
  ekf2_params->range_delay_ms = 5;
  ekf2_params->airspeed_delay_ms = 200;
  ekf2_params->ev_delay_ms = 175;
  ekf2_params->gyro_noise = 1.5e-2f;
  ekf2_params->accel_noise = 3.5e-1f;
  ekf2_params->gyro_bias_p_noise = 1.0e-3f;
  ekf2_params->accel_bias_p_noise = 3.0e-3f;
  ekf2_params->mage_p_noise = 1.0e-3f;
  ekf2_params->magb_p_noise = 1.0e-4f;
  ekf2_params->wind_vel_p_noise = 1.0e-1f;
  ekf2_params->terrain_p_noise = 5.0f;
  ekf2_params->terrain_gradient = 0.5f;
  ekf2_params->gps_vel_noise = 0.5f;
  ekf2_params->gps_pos_noise = 0.5f;
  ekf2_params->pos_noaid_noise = 10.0f;
  ekf2_params->baro_noise = 2.0f;
  ekf2_params->baro_innov_gate = 5.0f;
  ekf2_params->posNE_innov_gate = 5.0f;
  ekf2_params->vel_innov_gate = 5.0f;
  ekf2_params->tas_innov_gate = 3.0f;
  ekf2_params->mag_heading_noise = 0.3f;
  ekf2_params->mag_noise = 5.0e-2f;
  ekf2_params->eas_noise = 1.4f;
  ekf2_params->mag_declination_deg = 0;
  ekf2_params->heading_innov_gate = 2.6f;
  ekf2_params->mag_innov_gate = 3.0f;
  ekf2_params->mag_declination_source = 7;
  ekf2_params->mag_fusion_type = 0;
  ekf2_params->gps_check_mask = 21;
  ekf2_params->req_hacc = 5.0f;
  ekf2_params->req_vacc = 8.0f;
  ekf2_params->req_sacc = 1.0f;
  ekf2_params->req_nsats = 6;
  ekf2_params->req_gdop = 2.5f;
  ekf2_params->req_hdrift = 0.3f;
  ekf2_params->req_vdrift = 0.5f;
  ekf2_params->fusion_mode = 1;
  ekf2_params->vdist_sensor_type = 0;
  ekf2_params->range_noise = 0.1f;
  ekf2_params->range_innov_gate = 5.0f;
  ekf2_params->rng_gnd_clearance = 0.1f;
  ekf2_params->ev_innov_gate = 5.0f;
  ekf2_params->flow_noise = 0.15f;
  ekf2_params->flow_noise_qual_min = 0.5f;
  ekf2_params->flow_qual_min = 1;
  ekf2_params->flow_innov_gate = 3.0f;
  ekf2_params->flow_rate_max = 2.5f;
  ekf2_params->imu_pos_body(0) = 0.0f;
  ekf2_params->imu_pos_body(1) = 0.0f;
  ekf2_params->imu_pos_body(2) = 0.0f;
  ekf2_params->gps_pos_body(0) = 0.0f;
  ekf2_params->gps_pos_body(1) = 0.0f;
  ekf2_params->gps_pos_body(2) = 0.0f;
  ekf2_params->rng_pos_body(0) = 0.0f;
  ekf2_params->rng_pos_body(1) = 0.0f;
  ekf2_params->rng_pos_body(2) = 0.0f;
  ekf2_params->flow_pos_body(0) = 0.0f;
  ekf2_params->flow_pos_body(1) = 0.0f;
  ekf2_params->flow_pos_body(2) = 0.0f;
  ekf2_params->ev_pos_body(0) = 0.0f;
  ekf2_params->ev_pos_body(1) = 0.0f;
  ekf2_params->ev_pos_body(2) = 0.0f;
  ekf2_params->vel_Tau = 0.5f;
  ekf2_params->pos_Tau = 0.25f;
  ekf2_params->switch_on_gyro_bias = 0.1f;
  ekf2_params->switch_on_accel_bias = 0.2f;
  ekf2_params->initial_tilt_err = 0.1f;

  /* Initialize struct */
  ekf2.accel_stamp = 0;
  ekf2.gyro_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;


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

void ins_ekf2_update(void)
{
  if(ekf.update()) {
    /* Get the attitude */
    float att_q[4];
    struct FloatQuat ltp_to_imu_quat, ltp_to_body_quat;
    struct FloatQuat *body_to_imu_quat = orientationGetQuat_f(&ekf2.body_to_imu);
    ekf.copy_quaternion(att_q);
    ltp_to_imu_quat.qi = att_q[0];
    ltp_to_imu_quat.qx = att_q[1];
    ltp_to_imu_quat.qy = att_q[2];
    ltp_to_imu_quat.qz = att_q[3];

    // Rotate with respect to Body To IMU
    float_quat_comp_inv(&ltp_to_body_quat, &ltp_to_imu_quat, body_to_imu_quat);

    // Publish it to the state
    stateSetNedToBodyQuat_f(&ltp_to_body_quat);

    /* Get the body rates */
    float gyro_bias[3] = {};
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);
    ekf.get_gyro_bias(gyro_bias);
    FloatRates imu_rate, body_rate;
    imu_rate.p = ekf2.gyro.p - gyro_bias[0];
    imu_rate.q = ekf2.gyro.q - gyro_bias[1];
    imu_rate.r = ekf2.gyro.r - gyro_bias[2];

    // Rotate with respect to Body To IMU
    float_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &imu_rate);

    // Publish it to the state
    stateSetBodyRates_f(&body_rate);
  }
}

static void baro_cb(uint8_t __attribute__((unused)) sender_id, float pressure)
{

  /* no stamp available?
   * function call to ekf.setBaroData needs timestamp (or) zero
   * call function from air_data? float air_data_get_amsl(void)
   * or just use the variable that it returns directly? air_data.amsl_baro
   *
   * dont use the variable pressure?
   *
   * */

  //ekf.setBaroData(0, air_data.amsl_baro); /* is this the right function call statement*/

}

static void sonar_cb(uint8_t __attribute__((unused)) sender_id, float distance)
{

  /* which set function from ekf corresponds to sonar? */

}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  RATES_FLOAT_OF_BFP(ekf2.gyro, *gyro);
  if(ekf2.gyro_stamp > 0) {
    ekf2.gyro_dt = stamp - ekf2.gyro_stamp;
    ekf2.gyro_valid = true;
  }
  ekf2.gyro_stamp = stamp;

  if(ekf2.gyro_valid && ekf2.accel_valid) {
    float rates_int[3], accel_int[3];
    rates_int[0] = ekf2.gyro.p * ekf2.gyro_dt;
    rates_int[1] = ekf2.gyro.q * ekf2.gyro_dt;
    rates_int[2] = ekf2.gyro.r * ekf2.gyro_dt;
    accel_int[0] = ekf2.accel.x * ekf2.accel_dt;
    accel_int[1] = ekf2.accel.y * ekf2.accel_dt;
    accel_int[2] = ekf2.accel.z * ekf2.accel_dt;

    ekf.setIMUData(stamp, ekf2.gyro_dt * 1.e6f, ekf2.accel_dt * 1.e6f, rates_int, accel_int);
    ekf2.gyro_valid = false;
    ekf2.accel_valid = false;
  }
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
  ACCELS_FLOAT_OF_BFP(ekf2.accel, *accel);
  if(ekf2.accel_stamp > 0) {
    ekf2.accel_dt = stamp - ekf2.accel_stamp;
    ekf2.accel_valid = true;
  }
  ekf2.accel_stamp = stamp;

  if(ekf2.gyro_valid && ekf2.accel_valid) {
    float rates_int[3], accel_int[3];
    rates_int[0] = ekf2.gyro.p * ekf2.gyro_dt;
    rates_int[1] = ekf2.gyro.q * ekf2.gyro_dt;
    rates_int[2] = ekf2.gyro.r * ekf2.gyro_dt;
    accel_int[0] = ekf2.accel.x * ekf2.accel_dt;
    accel_int[1] = ekf2.accel.y * ekf2.accel_dt;
    accel_int[2] = ekf2.accel.z * ekf2.accel_dt;

    ekf.setIMUData(stamp, ekf2.gyro_dt * 1.e6f, ekf2.accel_dt * 1.e6f, rates_int, accel_int);
    ekf2.gyro_valid = false;
    ekf2.accel_valid = false;
  }
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t __attribute__((unused)) stamp,
                   struct Int32Vect3 *mag)
{

  float mag_r[3];
  FloatVect3 mag_f;
  MAGS_FLOAT_OF_BFP(mag_f, *mag);
  mag_r[0]=mag_f.x;
  mag_r[1]=mag_f.y;
  mag_r[2]=mag_f.z;
  ekf.setMagData(stamp, mag_r);

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
/*need to send a struct with the following members
 *          gps_msg.time_usec = gps.timestamp;  //unit of stamp?
			gps_msg.lat = gps.lat;              //latitude in 1E-7 degrees
			gps_msg.lon = gps.lon;              //longitude in 1E-7 degrees
			gps_msg.alt = gps.alt;              //altitude in 1E-3 meters (mm) above msl
			gps_msg.fix_type = gps.fix_type;    // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
			gps_msg.eph = gps.eph;              //GPS horizontal position accuracy in m
			gps_msg.epv = gps.epv;              //GPS vertical position accuracy in m
			gps_msg.sacc = gps.s_variance_m_s;  //GPS speed accuracy in m/s
			gps_msg.vel_m_s = gps.vel_m_s;      //GPS ground speed (m/s)
			gps_msg.vel_ned[0] = gps.vel_n_m_s; //GPS ground speed NED
			gps_msg.vel_ned[1] = gps.vel_e_m_s;
			gps_msg.vel_ned[2] = gps.vel_d_m_s;
			gps_msg.vel_ned_valid = gps.vel_ned_valid;  //GPS ground speed is valid
			gps_msg.nsats = gps.satellites_used;        //number of satellites used
			//TODO add gdop to gps topic
			gps_msg.gdop = 0.0f;   // geometric dilution of precision
*/
  //ekf.setGpsData(stamp, &gps_msg);

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
  orientationSetQuat_f(&ekf2.body_to_imu, q_b2i_f);
}

static void geo_mag_cb(uint8_t sender_id __attribute__((unused)), struct FloatVect3 *h)
{

}
