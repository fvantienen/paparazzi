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
#include "math/pprz_isa.h"

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
static abi_event gps_ev;
static abi_event body_to_imu_ev;
static abi_event opticflow_ekf_ev;

static void sonar_cb(uint8_t sender_id, uint32_t stamp, float distance);
static void baro_cb(uint8_t sender_id, uint32_t stamp, float pressure);
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void mag_cb(uint8_t sender_id, uint32_t stamp, struct FloatVect3 *mag);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void opticflow_ekf_cb(uint8_t sender_id, uint32_t stamp, float flow_x_integral, float flow_y_integral, uint8_t quality, float gyro_x_integral, float gyro_y_integral, float gyro_z_integral, uint32_t timespan);

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

    struct LtpDef_i ltp_def;

    struct OrientationReps body_to_imu;
    bool got_imu_data;
};
static struct ekf2_t ekf2;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  /*pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_int.ltp_pos.x, &ins_int.ltp_pos.y, &ins_int.ltp_pos.z,
                    &ins_int.ltp_speed.x, &ins_int.ltp_speed.y, &ins_int.ltp_speed.z,
                    &ins_int.ltp_accel.x, &ins_int.ltp_accel.y, &ins_int.ltp_accel.z);*/
}

static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  /*pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      &ins_int.baro_z, &ins_int.ltp_pos.z, &ins_int.ltp_speed.z, &ins_int.ltp_accel.z);*/
}

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
    float qfe = 101325.0; //TODO: this is qnh not qfe?
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ekf2.ltp_def.ecef.x, &ekf2.ltp_def.ecef.y, &ekf2.ltp_def.ecef.z,
                          &ekf2.ltp_def.lla.lat, &ekf2.ltp_def.lla.lon, &ekf2.ltp_def.lla.alt,
                          &ekf2.ltp_def.hmsl, &qfe);
}
#endif

void ins_ekf2_init(void)
{
  /* Get the ekf parameters */
  ekf2_params = ekf.getParamHandle();
  ekf2_params->fusion_mode = MASK_USE_GPS; //MASK_USE_OF uses only optic flow, use both for fusion
  ekf2_params->mag_declination_source = MASK_USE_GEO_DECL;
    /*
     * // Bit locations for mag_declination_source
#define MASK_USE_GEO_DECL   (1<<0)  // set to true to use the declination from the geo library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value
#define MASK_SAVE_GEO_DECL  (1<<1)  // set to true to set the EKF2_MAG_DECL parameter to the value returned by the geo library
#define MASK_FUSE_DECL      (1<<2)  // set to true if the declination is always fused as an observation to constrain drift when 3-axis fusion is performed

     */
  ekf2_params->mag_fusion_type = MAG_FUSE_TYPE_HEADING;
  ekf2_params->gps_check_mask = 21;
  /*not initialized in common.h*/ekf2_params->ev_innov_gate = 5.0f;

  /* Initialize struct */
  ekf2.accel_stamp = 0;
  ekf2.gyro_stamp = 0;
  ekf2.gyro_valid = false;
  ekf2.accel_valid = false;
  ekf2.got_imu_data = false;

/*#if USE_INS_NAV_INIT
  ins_init_origin_i_from_flightplan(&ins_int.ltp_def);
  ins_int.ltp_initialized = true;
#else
  ins_int.ltp_initialized  = false;
#endif*/


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

    /*/set filename for log file
    char filename[512];
    int i;
    sprintf(filename, "%s/ekf_log.csv", STRINGIFY(FILE_LOGGER_PATH));
    //open the log file
    file_logger = fopen(filename, "w");
    if (file_logger != NULL) {
        fprintf(file_logger, "Timestamp,");
        for(i=0; i < 32; i++)
        {
            fprintf(file_logger, "States_%d,", i);
        }
        for(i=0; i < 28; i++)
        {
            fprintf(file_logger, "Covariances_%d,", i);
        }
        for(i=0; i < 6; i++)
        {
            fprintf(file_logger, "vel_pos_innov_%d,", i);
        }
        for(i=0; i < 3; i++)
        {
            fprintf(file_logger, "mag_innov_%d,", i);
        }
        fprintf(file_logger, "heading_innov,");
        for(i=0; i < 6; i++)
        {
            fprintf(file_logger, "vel_pos_innov_var_%d,", i);
        }
        for(i=0; i < 3; i++)
        {
            fprintf(file_logger, "mag_innov_var_%d,", i);
        }
        fprintf(file_logger, "heading_innov_var\n");

    }*/

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgBARO_ABS(INS_EKF2_BARO_ID, &baro_ev, baro_cb);
  AbiBindMsgAGL(INS_EKF2_SONAR_ID, &sonar_ev, sonar_cb);
  AbiBindMsgIMU_GYRO_INT32(INS_EKF2_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_EKF2_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_GAUSS(INS_EKF2_MAG_ID, &mag_ev, mag_cb);
  AbiBindMsgGPS(INS_EKF2_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgOPTICAL_FLOW_EKF(ABI_BROADCAST, &opticflow_ekf_ev, opticflow_ekf_cb);
}

void ins_ekf2_update(void)
{
  if(ekf2.got_imu_data && ekf.update()) {
    /* Get the attitude */
    float att_q[4] = {};
    struct FloatQuat ltp_to_body_quat;
    ekf.copy_quaternion(att_q);
      ltp_to_body_quat.qi = att_q[0];
      ltp_to_body_quat.qx = att_q[1];
      ltp_to_body_quat.qy = att_q[2];
      ltp_to_body_quat.qz = att_q[3];

    // Publish it to the state
    stateSetNedToBodyQuat_f(&ltp_to_body_quat);

      /* Get the Body rates */
      struct FloatRates body_rates;
      float gyro_bias[3] = {};
      ekf.get_gyro_bias(gyro_bias);
      body_rates.p = ekf2.gyro.p - gyro_bias[0];
      body_rates.q = ekf2.gyro.q - gyro_bias[1];
      body_rates.r = ekf2.gyro.r - gyro_bias[2];

      //Publish to state
      stateSetBodyRates_f(&body_rates);

    /* Get the position */
    float pos_f[3] = {};
    struct NedCoor_f pos;
    ekf.get_position(pos_f);
    pos.x = pos_f[0];
    pos.y = pos_f[1];
    pos.z = pos_f[2];

    // Publish to the state
    stateSetPositionNed_f(&pos);

    /* Get the velocity */
     float vel_f[3] = {};
      struct NedCoor_f speed;
      ekf.get_velocity(vel_f);
      speed.x = vel_f[0];
      speed.y = vel_f[1];
      speed.z = vel_f[2];

     //Publish to state
    stateSetSpeedNed_f(&speed);

      /* Get the accelrations */
      struct NedCoor_f accel;
      float accel_bias[3];
      ekf.get_accel_bias(accel_bias);
      accel.x = ekf2.accel.x - accel_bias[0];
      accel.y = ekf2.accel.y - accel_bias[1];
      accel.z = ekf2.accel.z - accel_bias[2];

      //Publish to state
      stateSetAccelNed_f(&accel);

      // Get local origin
      // Position of local NED origin in GPS / WGS84 frame
      struct map_projection_reference_s ekf_origin = {};
      float ref_alt;
      struct LlaCoor_i lla_ref;
      uint64_t timestamp_orig;
      // true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
      ekf.get_ekf_origin(&timestamp_orig, &ekf_origin, &ref_alt);
      lla_ref.lat = ekf_origin.lat_rad * 180.0 / M_PI *1e7; // Reference point latitude in degrees
      lla_ref.lon = ekf_origin.lon_rad * 180.0 / M_PI *1e7; // Reference point longitude in degrees
      lla_ref.alt = ref_alt * 1000.0;
      ltp_def_from_lla_i(&ekf2.ltp_def, &lla_ref);

      /*/ states and ekf output Log
      uint32_t timestamp;
      float states[32] = {};
      float covariances[28] = {};
      float vel_pos_innov[6] = {};
      float mag_innov[3] = {};
      float heading_innov;
      float vel_pos_innov_var[6] = {};
      float mag_innov_var[3] = {};
      float heading_innov_var;

      timestamp = get_sys_time_usec();
      ekf.get_state_delayed(states);
      ekf.get_covariances(covariances);
      ekf.get_vel_pos_innov(vel_pos_innov);
      ekf.get_mag_innov(mag_innov);
      ekf.get_heading_innov(&heading_innov);
      ekf.get_vel_pos_innov_var(vel_pos_innov_var);
      ekf.get_mag_innov_var(mag_innov_var);
      ekf.get_heading_innov_var(&heading_innov_var);


      int s_len = 32;
      int c_len = 28;
      int v_len = 6;
      int m_len =3;
      int i;


      char log_msg[512000], number[50];
      snprintf(log_msg, 512000, "%d",timestamp);

      for(i=0; i < s_len; i++)
      {
          snprintf(number, 50, ",%.10f", states[i]);
          strcat(log_msg, number);

      }
      for(i=0; i < c_len; i++)
      {
          snprintf(number, 50, ",%.10f", covariances[i]);
          strcat(log_msg, number);
      }
      for(i=0; i < v_len; i++)
      {
          snprintf(number, 50, ",%.10f", vel_pos_innov[i]);
          strcat(log_msg, number);

      }
      for(i=0; i < m_len; i++)
      {
          snprintf(number, 50, ",%.10f", mag_innov[i]);
          strcat(log_msg, number);

      }
      snprintf(number, 50, ",%.10f", heading_innov);
      strcat(log_msg, number);
      for(i=0; i < v_len; i++)
      {
          snprintf(number, 50, ",%.10f", vel_pos_innov_var[i]);
          strcat(log_msg, number);
      }
      for(i=0; i < m_len; i++)
      {
          snprintf(number, 50, ",%.10f", mag_innov_var[i]);
          strcat(log_msg, number);
      }
      snprintf(number, 50, ",%.10f", heading_innov_var);
      strcat(log_msg, number);
      fprintf(file_logger, "%s\n", log_msg);

       */


  }

  ekf2.got_imu_data = false;
}

#include "mcu_periph/sys_time.h"
static void baro_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float pressure)
{
  /**
  * FROM: paparazzi/sw/airborne/math/pprz_isa.h
  * Get relative altitude from pressure (using simplified equation).
  * Given the current pressure and a reference pressure (at height=0),
  * calculate the height above the reference in meters.
  * If you pass QNH as reference pressure, you get the height above sea level.
  * Using QFE as reference pressure, you get height above the airfield.
  *
  * @param pressure current pressure in Pascal (Pa)
  * @param ref_p reference pressure (QFE) when height=0 or QNH at sea level
  * @return height in m above reference in ISA conditions
  */
   float height_amsl_m = pprz_isa_height_of_pressure(pressure, 101325.0); //101325.0 defined as PPRZ_ISA_SEA_LEVEL_PRESSURE in pprz_isa.h
   ekf.setBaroData(stamp, &height_amsl_m);

}

static void sonar_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp, float distance)
{
  ekf.setRangeData(stamp, &distance);

}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
    FloatRates imu_rate;
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);
  RATES_FLOAT_OF_BFP(imu_rate, *gyro);

    // Rotate with respect to Body To IMU
    float_rmat_transp_ratemult(&ekf2.gyro, body_to_imu_rmat, &imu_rate);

  if(ekf2.gyro_stamp > 0) {
    ekf2.gyro_dt = stamp - ekf2.gyro_stamp;
    ekf2.gyro_valid = true;
  }
  ekf2.gyro_stamp = stamp;

  if(ekf2.gyro_valid && ekf2.accel_valid) {
    float rates_int[3], accel_int[3];
    rates_int[0] = ekf2.gyro.p * ekf2.gyro_dt/1.e6f;
    rates_int[1] = ekf2.gyro.q * ekf2.gyro_dt/1.e6f;
    rates_int[2] = ekf2.gyro.r * ekf2.gyro_dt/1.e6f;
    accel_int[0] = ekf2.accel.x * ekf2.accel_dt/1.e6f;
    accel_int[1] = ekf2.accel.y * ekf2.accel_dt/1.e6f;
    accel_int[2] = ekf2.accel.z * ekf2.accel_dt/1.e6f;

    ekf.setIMUData(stamp, ekf2.gyro_dt, ekf2.accel_dt, rates_int, accel_int);
    ekf2.gyro_valid = false;
    ekf2.accel_valid = false;
    ekf2.got_imu_data = true;
  }
}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp, struct Int32Vect3 *accel)
{
    struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);

    struct FloatVect3 accel_imu;
  ACCELS_FLOAT_OF_BFP(accel_imu, *accel);

    float_rmat_transp_vmult(&ekf2.accel, body_to_imu_rmat, &accel_imu);

  if(ekf2.accel_stamp > 0) {
    ekf2.accel_dt = stamp - ekf2.accel_stamp;
    ekf2.accel_valid = true;
  }
  ekf2.accel_stamp = stamp;

  if(ekf2.gyro_valid && ekf2.accel_valid) {
    float rates_int[3], accel_int[3];
    rates_int[0] = ekf2.gyro.p * ekf2.gyro_dt/1.e6f;
    rates_int[1] = ekf2.gyro.q * ekf2.gyro_dt/1.e6f;
    rates_int[2] = ekf2.gyro.r * ekf2.gyro_dt/1.e6f;
    accel_int[0] = ekf2.accel.x * ekf2.accel_dt/1.e6f;
    accel_int[1] = ekf2.accel.y * ekf2.accel_dt/1.e6f;
    accel_int[2] = ekf2.accel.z * ekf2.accel_dt/1.e6f;

    ekf.setIMUData(stamp, ekf2.gyro_dt, ekf2.accel_dt, rates_int, accel_int);
    ekf2.gyro_valid = false;
    ekf2.accel_valid = false;
    ekf2.got_imu_data = true;
  }
}

static void mag_cb(uint8_t __attribute__((unused)) sender_id,
                   uint32_t stamp,
                   struct FloatVect3 *mag)
{
  float mag_r[3];
  struct FloatRMat *body_to_imu_rmat = orientationGetRMat_f(&ekf2.body_to_imu);
  struct FloatVect3 mag_gauss;
  float_rmat_transp_vmult(&mag_gauss, body_to_imu_rmat, mag);
  mag_r[0] = mag_gauss.x;
  mag_r[1] = mag_gauss.y;
  mag_r[2] = mag_gauss.z;

  ekf.setMagData(stamp, mag_r);
    //CHECKING
    ekf2.got_imu_data = true;
}


static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{
    /*
     * struct gps_message {
	uint64_t time_usec;
	int32_t lat;                // Latitude in 1E-7 degrees
	int32_t lon;                // Longitude in 1E-7 degrees
	int32_t alt;                // Altitude in 1E-3 meters (millimeters) above MSL
	uint8_t fix_type;           // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	float eph;                  // GPS horizontal position accuracy in m
	float epv;                  // GPS vertical position accuracy in m
	float sacc;                 // GPS speed accuracy in m/s
	float vel_m_s;              // GPS ground speed (m/s)
	float vel_ned[3];           // GPS ground speed NED (m/s)
	bool vel_ned_valid;         // GPS ground speed is valid
	uint8_t nsats;              // number of satellites used
	float gdop;                 // geometric dilution of precision
};
     */
  struct gps_message gps_msg = {};
  gps_msg.time_usec = stamp;
  gps_msg.lat = gps_s->lla_pos.lat;
  gps_msg.lon = gps_s->lla_pos.lon;
  gps_msg.alt = gps_s->hmsl;
  gps_msg.fix_type = gps_s->fix;
  gps_msg.eph = gps_s->hacc/1000.0;
  gps_msg.epv = gps_s->vacc/1000.0;
  gps_msg.sacc = gps_s->sacc/100.0;
  gps_msg.vel_m_s = gps_s->gspeed/100.0;
  gps_msg.vel_ned[0] = (gps_s->ned_vel.x)/100.0;//SPEED_FLOAT_OF_BFP(gps_s->ned_vel.x)/100.0;
  gps_msg.vel_ned[1] = (gps_s->ned_vel.y)/100.0;//SPEED_FLOAT_OF_BFP(gps_s->ned_vel.y)/100.0;
  gps_msg.vel_ned[2] = (gps_s->ned_vel.z)/100.0;//SPEED_FLOAT_OF_BFP(gps_s->ned_vel.z)/100.0;
  gps_msg.vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  gps_msg.nsats = gps_s->num_sv;
  //TODO add gdop to gps topic
  gps_msg.gdop = 0.0f;

  ekf.setGpsData(stamp, &gps_msg);
}

static void opticflow_ekf_cb(uint8_t sender_id,
                             uint32_t stamp,
                             float flow_x_integral,
                             float flow_y_integral,
                             uint8_t quality,
                             float gyro_x_integral,
                             float gyro_y_integral,
                             float gyro_z_integral,
                             uint32_t timespan)
{

/*
 *
struct flow_message {
  uint8_t quality;			// Quality of Flow data
  Vector2f flowdata;			// Flow data received
  Vector3f gyrodata;			// Gyro data from flow sensor
  uint32_t dt;				// integration time of flow samples
};

////

 struct flowSample {
  uint8_t  quality; // quality indicator between 0 and 255
  Vector2f flowRadXY; // measured delta angle of the image about the X and Y body axes (rad), RH rotaton is positive
  Vector2f flowRadXYcomp;	// measured delta angle of the image about the X and Y body axes after removal of body rotation (rad), RH rotation is positive
  Vector3f gyroXYZ; // measured delta angle of the inertial frame about the body axes obtained from rate gyro measurements (rad), RH rotation is positive
  float    dt; // amount of integration time (sec)
  uint64_t time_us; // timestamp in microseconds of the integration period mid-point
};


//////

if (optical_flow_updated) {
          flow_message flow;


          if (PX4_ISFINITE(optical_flow.pixel_flow_y_integral) &&
              PX4_ISFINITE(optical_flow.pixel_flow_x_integral)) {
              _ekf.setOpticalFlowData(optical_flow.timestamp, &flow);
          }
      }

 *
 *
 */

    flow_message flow;
    flow.flowdata(0) = flow_x_integral;      // accumulated optical flow in radians around x axis
    flow.flowdata(1) = flow_y_integral;
    flow.quality = quality;
    flow.gyrodata(0) = gyro_x_integral;
    flow.gyrodata(1) = gyro_y_integral;
    flow.gyrodata(2) = gyro_z_integral;
    flow.dt = timespan;

    ekf.setOpticalFlowData(stamp, &flow);

}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&ekf2.body_to_imu, q_b2i_f);
}
