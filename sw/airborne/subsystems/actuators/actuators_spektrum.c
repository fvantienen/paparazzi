/*
 * Copyright (C) 2013 The Paparazzi Team
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

/** @file actuators_asctec_v2.c
 *  Actuators driver for Asctec v2 motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_spektrum.h"

#include "mcu_periph/uart.h"

struct ActuatorsSpektrum actuators_spektrum;

void actuators_asctec_v2_init(void)
{
  actuators_spektrum.device = &((ACUTATORS_SPEKTRUM_OUT).link_device);
}


void actuators_asctec_v2_set(void)
{
  actuators_spektrum.device.put_byte(0x01); // 7 channels, 11 bit
}

