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
float kalamos_search_height = 35.0;
float kalamos_descend_height = 30.0;
float kalamos_height_gain = 1.0;
bool kalamos_enable_landing = false;
bool kalamos_enable_spotsearch = false;
float kalamos_landing_decent_speed = 0.05;
float kalamos_pos_gain = 0.05;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void kalamos_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DEBUG(trans, dev, AC_ID, sizeof(struct Kalamos2PPRZPackage), (unsigned char *) &k2p_package);
}
#endif

/* Initialize the Kalamos */
void kalamos_init() {
  // Initialize transport protocol
  pprz_transport_init(&kalamos.transport);


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG, kalamos_raw_downlink);
#endif

  NavSetWaypointHere(WP_LANDING);
  k2p_package.height = -0.01;
  k2p_package.status = 1;


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
    timeoutcount = 100;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();

    float diff_landing = (kalamos_descend_height - k2p_package.height)*kalamos_height_gain;
    float pprzheight_landing  = pos->z + diff_landing;

    waypoint_set_alt(WP_LANDING,pprzheight_landing);

    float diff_search = (kalamos_search_height - k2p_package.height)*kalamos_height_gain;
    float pprzheight_search  = pos->z + diff_search;
    waypoint_set_alt(WP_JOE,pprzheight_search);

    if (kalamos_enable_spotsearch) {
      waypoint_set_xy_i(WP_LANDING,POS_BFP_OF_REAL(k2p_package.land_gpsx), POS_BFP_OF_REAL(k2p_package.land_gpsy));
    }

    if (k2p_package.height < 5.0) {
        kalamos_enable_landing = false;
    }

    if (kalamos_enable_landing && timeoutcount > 0) {
      if (k2p_package.min_height > 5.0) {
        kalamos_descend_height -= kalamos_landing_decent_speed;
      }


      float bx = (k2p_package.target_x * kalamos_pos_gain); // x is right
      float by = k2p_package.target_y * kalamos_pos_gain; // y is nose

      float psi = stateGetNedToBodyEulers_f()->psi;

      float target_x = waypoint_get_x(WP_LANDING) + cos(psi) * bx + sin(psi) * by;
      float target_y = waypoint_get_y(WP_LANDING) + -sin(psi) * bx + cos(psi) * by;

      waypoint_set_xy_i(WP_LANDING,POS_BFP_OF_REAL(target_x), POS_BFP_OF_REAL(target_y));

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

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();


  struct PPRZ2KalamosPackage p2k_package;
  p2k_package.phi = att->phi;
  p2k_package.theta = att->theta;
  p2k_package.psi = att->psi;
  p2k_package.gpsx = pos->x;
  p2k_package.gpsy = pos->y;
  p2k_package.gpsz = gps.lla_pos.alt;
  p2k_package.enables = 0;
  if (kalamos_enable_landing)
    p2k_package.enables |= 0b1;
  if (kalamos_enable_spotsearch)
    p2k_package.enables |= 0b10;

  if (timeoutcount > 0) {
    timeoutcount--;
  } else {
    k2p_package.status = 1;
  }
  uint16_t status = k2p_package.status;

  // Send Telemetry report
  DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &status, &k2p_package.height);


  pprz_msg_send_IMCU_DEBUG(&(kalamos.transport.trans_tx), kalamos.device,
                                         1, sizeof(struct PPRZ2KalamosPackage), (unsigned char *)(&p2k_package));
}

void enableLandingspotSearch(bool b) {
  kalamos_enable_spotsearch = b;
}

void enableKalamosDescent(bool b) {
  kalamos_enable_landing = b;
}
