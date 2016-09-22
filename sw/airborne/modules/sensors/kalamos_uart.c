/*
 * Copyright (C) C. De Wagter
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/sensors/kalamos_uart.c"
 * @author C. De Wagter
 * Parrot Kalamos Nvidia tk1 stereo vision uart (RS232) communication
 */

#include "modules/sensors/kalamos_uart.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "state.h"
#include "subsystems/navigation/waypoints.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/gps.h"

#include "generated/flight_plan.h"

 /* Main magneto structure */
static struct kalamos_t kalamos = {
  .device = (&((KALAMOS_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Kalamos

struct  Kalamos2PPRZPackage k2p_package;
bool kalamos_enable_landing = false;
bool kalamos_enable_spotsearch = false;
bool kalamos_enable_findjoe = false;
bool kalamos_enable_opticflow = false;
float kalamos_search_height = 35.0;
float kalamos_land_xy_gain = 1.5f;
float kalamos_land_z_gain = 0.5f;
struct FloatVect3 land_cmd;


int32_t zeroheight;
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void kalamos_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DEBUG(trans, dev, AC_ID, sizeof(struct Kalamos2PPRZPackage), (unsigned char *) &k2p_package);
}

static void send_kalamos(struct transport_tx *trans, struct link_device *dev)
{
  char hoertje = k2p_package.status;
  hoertje+=48;
  float dummy = 0.0;
  pprz_msg_send_KALAMOS(trans, dev, AC_ID,
                        &hoertje,
                        &k2p_package.height,
                        &k2p_package.avoid_psi,
                        &k2p_package.avoid_rate,
                        &k2p_package.descend_z,
                        &k2p_package.joe_enu_x,
                        &k2p_package.joe_enu_y,
                        &k2p_package.land_enu_x,
                        &k2p_package.land_enu_y,
                        &k2p_package.flow_x,
                        &k2p_package.flow_y,
                        &dummy,
                        &dummy);
}
#endif

/* Initialize the Kalamos */
void kalamos_init() {
  // Initialize transport protocol
  pprz_transport_init(&kalamos.transport);


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG, kalamos_raw_downlink);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_KALAMOS, send_kalamos);
#endif

  NavSetWaypointHere(WP__LANDING);
  k2p_package.height = -0.01;
  k2p_package.status = 1;

  land_cmd.x = 0;
  land_cmd.y = 0;
  land_cmd.z = 0;
}

static int timeoutcount = 0;

/* Parse the InterMCU message */
static inline void kalamos_parse_msg(void)
{

  /* Parse the kalamos message */
  uint8_t msg_id = mp_msg_buf[1];

  switch (msg_id) {

  /* Got a kalamos message */
  case DL_IMCU_DEBUG: {
    uint8_t size = DL_IMCU_DEBUG_msg_length(mp_msg_buf);
    uint8_t *msg = DL_IMCU_DEBUG_msg(mp_msg_buf);

    unsigned char * tmp = (unsigned char*)&k2p_package;
    for(uint8_t i = 0; i < size; i++) {
      tmp[i] = msg[i];
    }
    timeoutcount = 1000;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();

    //float diff_search = (kalamos_search_height - k2p_package.height)*kalamos_height_gain;

    if (kalamos_enable_spotsearch) {
      waypoint_set_xy_i(WP_LANDSPOT, POS_BFP_OF_REAL(k2p_package.land_enu_x), POS_BFP_OF_REAL(k2p_package.land_enu_y));
    }

    if (kalamos_enable_landing) {
/*
      struct FloatQuat *att = stateGetNedToBodyQuat_f();

      struct FloatRMat ltp_to_kalamos_rmat;
      float_rmat_of_quat(&ltp_to_kalamos_rmat, att);

      //x,y,z pos van joe
      struct FloatVect3 joe;
      joe.x = k2p_package.target_x;
      joe.y = k2p_package.target_y;
      joe.z = k2p_package.height;

      struct FloatVect3 measured_ltp;
      float_rmat_transp_vmult(&measured_ltp, &ltp_to_kalamos_rmat, &joe);

      waypoint_set_xy_i(WP__LANDING,POS_BFP_OF_REAL(measured_ltp.x), POS_BFP_OF_REAL(measured_ltp.y));
      */

      land_cmd.x = k2p_package.descend_x * kalamos_land_xy_gain;
      land_cmd.y = k2p_package.descend_y * kalamos_land_xy_gain;
      land_cmd.z = -k2p_package.descend_z * kalamos_land_z_gain;

      float psi = stateGetNedToBodyEulers_f()->psi;
      float heading_to_go = psi + k2p_package.avoid_psi + M_PI;
      FLOAT_ANGLE_NORMALIZE(heading_to_go);

      struct EnuCoor_f target;
      target.x = pos->x + sin(heading_to_go)*k2p_package.avoid_rate*kalamos_land_xy_gain;
      target.y = pos->y + cos(heading_to_go)*k2p_package.avoid_rate*kalamos_land_xy_gain;
      target.z = waypoint_get_alt(WP__LANDING);

      if((kalamos_land_xy_gain > 0.001) && (k2p_package.avoid_rate > 0.2))
        waypoint_set_enu(WP__LANDING, &target);
    }

    if (kalamos_enable_findjoe) {
      waypoint_set_xy_i(WP_JOE,POS_BFP_OF_REAL(k2p_package.joe_enu_x), POS_BFP_OF_REAL(k2p_package.joe_enu_y));

      uint8_t wp_id = WP_JOE;
      DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id,&(waypoints[WP_JOE].enu_i.x),
                                 &(waypoints[WP_JOE].enu_i.y), &(waypoints[WP_JOE].enu_i.z));
    }

    // Send ABI message
    if (timeoutcount > 0) {
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, k2p_package.height);
    }

    break;
  }
    default:
      break;
  }
}

/* We need to wait for incomming messages */
void kalamos_event() {
  // Check if we got some message from the Kalamos
  pprz_check_and_parse(kalamos.device, &kalamos.transport, mp_msg_buf, &kalamos.msg_available);

  // If we have a message we should parse it
  if (kalamos.msg_available) {
    kalamos_parse_msg();
    kalamos.msg_available = false;
  }
}

void kalamos_periodic() {


  struct FloatEulers *attE = stateGetNedToBodyEulers_f();
  struct FloatQuat *att = stateGetNedToBodyQuat_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();


  struct PPRZ2KalamosPackage p2k_package;
  p2k_package.phi = attE->phi;
  p2k_package.theta = attE->theta;
  p2k_package.psi = attE->psi;
  p2k_package.qi = att->qi;
  p2k_package.qx = att->qx;
  p2k_package.qy = att->qy;
  p2k_package.qz = att->qz;
  p2k_package.gpsx = pos->x;
  p2k_package.gpsy = pos->y;
  p2k_package.gpsz = pos->z;
  p2k_package.enables = 0;
  if (kalamos_enable_landing)
    p2k_package.enables |= 0b1;
  if (kalamos_enable_spotsearch)
    p2k_package.enables |= 0b10;
  if (kalamos_enable_findjoe)
    p2k_package.enables |= 0b100;
if (kalamos_enable_opticflow)
    p2k_package.enables |= 0b1000;
  if (timeoutcount > 0) {
    timeoutcount--;
  } else {
    k2p_package.status = 1;
  }

  // Send Telemetry report
  //char hoertje = k2p_package.status;
  //hoertje+=48; //fuck you pprz
  //DOWNLINK_SEND_KALAMOS(DefaultChannel, DefaultDevice, &hoertje, &k2p_package.height,&k2p_package.avoid_psi,&k2p_package.avoid_rate,&k2p_package.descend_z,&k2p_package.joe_enu_x,&k2p_package.joe_enu_y,&k2p_package.land_enu_x,&k2p_package.land_enu_y,&k2p_package.flow_x,&k2p_package.flow_y);


  pprz_msg_send_IMCU_DEBUG(&(kalamos.transport.trans_tx), kalamos.device,
                                         1, sizeof(struct PPRZ2KalamosPackage), (unsigned char *)(&p2k_package));
}

void enableKalamosLandingspotSearch(bool b) {
  kalamos_enable_spotsearch = b;
}

void enableKalamosDescent(bool b) {
  kalamos_enable_landing = b;
}

void enableKalamosFindJoe(bool b) {
  kalamos_enable_findjoe = b;
}

void enableKalamosOpticFlow(bool b) {
  kalamos_enable_opticflow = b;
}

void setZeroHeight(void){
  zeroheight = gps.hmsl;
}

