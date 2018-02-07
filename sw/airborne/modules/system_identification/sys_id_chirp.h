/*
 * Copyright (C) Joost Meulenbeld
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
 * @file "modules/helicopter/sys_id_chirp.h"
 * @author Joost Meulenbeld
 * System identification chirp
 */

#ifndef SYS_ID_CHIRP_H
#define SYS_ID_CHIRP_H

#include <std.h>
#include <stdbool.h>
#include "modules/system_identification/pprz_chirp.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

// Number of axes for which the chirp will generate a signal
#define CHIRP_NO_AXES 5

// The current values for all axes that should be added to output commands to the UAV
extern int32_t current_chirp_values[CHIRP_NO_AXES];

extern uint8_t chirp_active;
extern int32_t chirp_amplitude;
extern float chirp_noise_stdv_onaxis_ratio;
extern float chirp_noise_stdv_offaxis_ratio;

// 0: Roll, 1: Pitch, 2: Yaw, 3: Elevator, 4: Aileron
extern uint8_t chirp_axis;

void sys_id_chirp_init(void);

// If chirp is running, update its values
extern void sys_id_chirp_run(void);

// Handler for chaning the chirp_active variable in the GCS
extern void sys_id_chirp_chirp_activate_handler(uint8_t activate);

#endif // SYS_ID_CHIRP_H
