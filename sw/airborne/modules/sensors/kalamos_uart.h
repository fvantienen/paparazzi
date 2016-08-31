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
 * @file "modules/sensors/kalamos_uart.h"
 * @author Kevin van Hecke
 * Parrot Kalamos Nvidia tk1 stereo vision uart (RS232) communication
 */

#ifndef KALAMOS_UART_H
#define KALAMOS_UART_H

#include "std.h"
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"
#include "math/pprz_orientation_conversion.h"


/* Main kalamos structure */
struct kalamos_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  struct OrientationReps imu_to_mag;    ///< IMU to magneto translation
  bool msg_available;                 ///< If we received a message
};


//should be exactly the same as pprz.h
struct Kalamos2PPRZPackage {
    float height;
    float min_height;
    float target_x;
    float target_y;
    float land_gpsx;
    float land_gpsy;
    float flow_x;
    float flow_y;
    char status;
};
extern struct Kalamos2PPRZPackage k2p_package;

//should be exactly the same as pprz.h
struct PPRZ2KalamosPackage {
    float phi;
    float theta;
    float psi;
    float gpsx;
    float gpsy;
    float gpsz;
    unsigned char enables;
};

extern float kalamos_descend_height;
extern float kalamos_search_height;
extern float kalamos_height_gain;
extern bool kalamos_enable_landing ;
extern float kalamos_landing_decent_speed ;
extern float kalamos_pos_gain;
extern bool kalamos_enable_spotsearch ;

extern void kalamos_init(void);
extern void kalamos_event(void);
extern void kalamos_periodic(void);

extern void enableLandingspotSearch(bool b);
extern void enableKalamosDescent(bool b);

#endif

