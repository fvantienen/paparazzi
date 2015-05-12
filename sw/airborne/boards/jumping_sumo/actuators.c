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
 * @file boards/jumping_sumo/actuators.c
 * Actuator driver for the Jumping Sumo
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/motor_mixing.h"
#include "subsystems/electrical.h"
#include "actuators.h"
#include "autopilot.h"

struct ActuatorsJumpingSumo actuators_jumping_sumo;

void actuators_jumping_sumo_init(void)
{
  /* Initialize the I2C connection */

}

void actuators_jumping_sumo_commit(void)
{

}
