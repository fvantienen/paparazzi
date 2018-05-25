/*
 * Copyright (C) 2018 Papparazzi Team
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
 * @file "modules/alpha_esc/alpha_esc.c"
 * @author D.C. van Wijngaarden and I.Z.El-Hajj
 * Read the default data send from T-motorS Alpha ESC into the autopilot so it can be used for needed purpose
 */

#include "modules/esc/alpha_esc.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"

/* Main Alpha ESC structure */
static struct alpha_esc_t alpha_esc = {
  .device = (&((ALPHA_ESC_PORT).device)),
  .msg_available = false
};
static uint8_t alpha_esc_msg_buf[24]  __attribute__((aligned));   ///< The message buffer for the Alpha ESC chosen to be 2*message_size

static struct esc_data alpha_esc_data;
static const uint8_t tempTable[ 220 ] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,16,17,18,19,19,20,21,21,22,23,23,24,25,25,26,26,27,28,28,29,29,30,30,
			  31,32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,40,40,41,41,42,42,43,43,44,44,44,45,45,46,46,47,47,48,48,49,49,50,50,50,51,51,52,52,53,53,
			  53,54,54,55,55,56,56,57,57,58,58,58,59,59,60,60,61,61,61,62,62,63,63,64,64,65,65,66,66,66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,74,74,75,
			  75,75,76,77,77,78,78,79,79,80,80,81,81,82,82,83,83,84,85,85,86,86,86,87,88,88,89,90,90,91,92,92,93,94,95,95,96,96,96,97,98,98,99,100,101,101,
			  102,103,103,104,105,106,106,107,107,108,109,110,110,111,112,113,113,114,115,115,116,117,117,118,119,120,120,121,122,122,123,124,125,125,126,
			  127,127,128,129,129};

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void alpha_esc_downlink(struct transport_tx *trans, struct link_device *dev)
{

	  pprz_msg_send_ALPHA_ESC(trans, dev, AC_ID, &alpha_esc_data.bale_no, &alpha_esc_data.rx_throttle, &alpha_esc_data.output_throttle,
			  &alpha_esc_data.rpm, &alpha_esc_data.voltage, &alpha_esc_data.busbar_current, &alpha_esc_data.phase_wire_current, &alpha_esc_data.mosfet_temp,
			  &alpha_esc_data.capacitor_temp, &alpha_esc_data.status_code,  &alpha_esc_data.verify_code );
}
#endif

uint16_t get_rpm_alpha_esc(void) {
	return alpha_esc_data.rpm;
}


/* Initialize the magneto and pitot */
void alpha_esc_init() {

	pprz_transport_init(&alpha_esc.transport);
	//uart_periph_set_baudrate(&ALPHA_ESC_PORT, B19200);
	alpha_esc_data.buffer_counter = 0;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ALPHA_ESC, alpha_esc_downlink);
#endif
}

/* Parse the InterMCU message */
void alpha_esc_parse_msg(void)
{
	// Dummy variables to apply the bitshifting later (for scaling according to protocol)
	  uint16_t rx_throttle_dummy = (alpha_esc_msg_buf[6] <<8 | alpha_esc_msg_buf[7]);
	  uint16_t output_throttle_dummy = (alpha_esc_msg_buf[8] <<8 | alpha_esc_msg_buf[9]);
	  uint16_t rpm_dummy = (alpha_esc_msg_buf[10] <<8 | alpha_esc_msg_buf[11]);

	  // Read the alpha esc information
	  alpha_esc_data.bale_no = (uint16_t)((alpha_esc_msg_buf[4] <<8 | alpha_esc_msg_buf[5]));
	  alpha_esc_data.rx_throttle = (uint16_t)((rx_throttle_dummy)*100/1024); // Original (uint16_t)((alpha_esc_msg_buf[6] <<8 | alpha_esc_msg_buf[7])*100/1024);
	  alpha_esc_data.output_throttle = (uint16_t)((output_throttle_dummy)*100/1024); // Original (uint16_t)((alpha_esc_msg_buf[8] <<8 | alpha_esc_msg_buf[9])*100/1024);
	  alpha_esc_data.rpm = (uint16_t)(rpm_dummy * 10.f / 21); // (uint16_t)((alpha_esc_msg_buf[10] <<8 | alpha_esc_msg_buf[11])*10/108);
	  alpha_esc_data.voltage = ((uint16_t)((alpha_esc_msg_buf[12] <<8 | alpha_esc_msg_buf[13]))); // Needs to be divided by 10 to get real voltage
	  alpha_esc_data.busbar_current = ((int16_t)((alpha_esc_msg_buf[14] <<8 | alpha_esc_msg_buf[15])));// Needs to be divided by 64
	  alpha_esc_data.phase_wire_current = ((int16_t)((alpha_esc_msg_buf[16] <<8 | alpha_esc_msg_buf[17]))); //Needs to be divided by 64
	  alpha_esc_data.mosfet_temp = tempTable[alpha_esc_msg_buf[18]];
	  alpha_esc_data.capacitor_temp = tempTable[alpha_esc_msg_buf[19]];
	  alpha_esc_data.status_code = (uint16_t)((alpha_esc_msg_buf[20] <<8 | alpha_esc_msg_buf[21]));
	  alpha_esc_data.verify_code = (uint16_t)((alpha_esc_msg_buf[22] <<8 | alpha_esc_msg_buf[23]));// shift the right byte instead of the
}

/* We need to wait for incoming messages */
void alpha_esc_event() {
	if(!uart_char_available(&ALPHA_ESC_PORT))
		return;
	
	uint8_t rcv_byte = uart_getch(&ALPHA_ESC_PORT);
	if (rcv_byte == START_BYTE || alpha_esc_data.buffer_counter > 0) {
		alpha_esc_msg_buf[alpha_esc_data.buffer_counter] = rcv_byte;
		alpha_esc_data.buffer_counter++;
	}

	if (alpha_esc_data.buffer_counter > 23){
		uint8_t checksum = 0;
		for(uint8_t i = 0; i < 22; i++)
			checksum += alpha_esc_msg_buf[i];


		uint8_t recv_checksum = (uint16_t)((alpha_esc_msg_buf[22]) | (alpha_esc_msg_buf[23]<<8));
		if ((alpha_esc_msg_buf[1]==22) && (alpha_esc_msg_buf[2]== 1) && (alpha_esc_msg_buf[3]== 2) && (recv_checksum == checksum) )
			alpha_esc_parse_msg();

		alpha_esc_data.buffer_counter = 0;
	}
}

