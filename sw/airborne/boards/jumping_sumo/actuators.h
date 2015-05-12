/*
 *
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file boards/jumping_sumo/actuators.h
 * Actuator driver for the Jumping Sumo
 */

#ifndef ACTUATORS_JUMPING_SUMO_H_
#define ACTUATORS_JUMPING_SUMO_H_

#include <stdint.h>

struct ActuatorsJumpingSumo {
  uint16_t pwm_ref[4];                ///< Reference PWM
};

#define ActuatorsJumpingSumoSet(_i, _v) { actuators_jumping_sumo.pwm_ref[_i] = _v; }
#define ActuatorsJumpingSumoCommit() actuators_jumping_sumo_commit();
#define ActuatorsJumpingSumoInit() actuators_humping_sumo_init();

extern struct ActuatorsJumpingSumo actuators_jumping_sumo;
extern void actuators_jumping_sumo_commit(void);
extern void actuators_jumping_sumo_init(void);

#endif /* ACTUATORS_BEBOP_RAW_H_ */
