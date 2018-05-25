/*
 * Copyright (C) Paparazzi Team
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
 * @file "modules/alpha_esc/alpha_esc.h"
 * @author D.C. van Wijngaarden and I.Z.El-Hajj
 * Converts telemtry data from t motor alpha esc to the autopilot (and groundstation)
 */

#ifndef ALPHA_ESC_H
#define ALPHA_ESC_H

#define START_BYTE 0x9B  //1st start block identifier byte
#define SECOND_BYTE 0x16 //2nd Start block identifier byte
#define THIRD_BYTE 0x01  //3rd Start block identifier byte
#define FOURTH_BYTE 0x02 //4rd Start block identifier byte

#define MESSAGE_LENGTH 24 // 24 bytes is the message length according to the protocol def

#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

/* Main Alpha ESC strcuture */
struct alpha_esc_t {
  struct link_device *device;           ///< The device which is used for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                 ///< If we received a message
};

struct esc_data {
	uint16_t bale_no;
	uint16_t rx_throttle;
	uint16_t output_throttle;
	uint16_t rpm;
	uint16_t voltage;
	int16_t busbar_current; // data type might need to be changed later
	int16_t phase_wire_current;
	uint8_t mosfet_temp;// data type might need to be changed later
	uint8_t capacitor_temp;// data type might need to be changed later
	uint16_t status_code;
	uint16_t verify_code;
	uint8_t buffer_counter ;
};

uint16_t get_rpm_alpha_esc(void);

extern void alpha_esc_init(void);
extern void alpha_esc_event(void);

extern void alpha_esc_parse_msg(void);

#endif

