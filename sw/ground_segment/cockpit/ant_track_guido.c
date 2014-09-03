/*
 * Paparazzi $Id: ant_track_pmm.c 2010-11-27 18:14:20Z griffin $
 *
 * Copyright (C) 2010
 *
 * Modified by: Mark Griffin and Todd Sandercock
 * Modified by: Chris Efstathiou for the Pololu Micro Mestro usb servo controller Jun/2010
 * Added command line options, a 360 pan option and "MANUAL" control.
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
 *
 * The antenna tracker zero azimuth is to the NORTH (NORTH = 0, EAST = 90 WEST = -90, SOUTH = 180/0 degrees).
 * The elevation zero is totally horizontal, 90 is up and 180 is to the back.
 * The servo used must be able to do 180 degrees in order to get full 360 degree coverage from the tracker.
 */
#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <termios.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

#define POLOLU_PROTOCOL_START          0xAA
#define POLOLU_BOARD_ID                12
#define SET_SERVO_POSITION_COMMAND     0x04
#define SET_SERVO_SPEED_COMMAND        0x07
#define SET_SERVO_ACCELERATION_COMMAND 0x09
#define SET_SERVO_CENTER_COMMAND       0x22


#define SERIAL_BUFFER_SIZE 20

#define MANUAL 0
#define AUTO 1

#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/termios.h>
#include <sys/ioctl.h>

#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>

static int baudrates[] = { B0, B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400 };


/****************************************************************************/
/* Open serial device for requested protocoll */
/****************************************************************************/
int fd; /* File descriptor for the port */
int c_init_serial()
{
  struct termios orig_termios, cur_termios;

  int br = B38400;

  fd = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NONBLOCK);

  if (fd == -1) printf("opening modem serial device : fd < 0");

  if (tcgetattr(fd, &orig_termios)) printf("getting modem serial device attr");
  cur_termios = orig_termios;

  /* input modes - turn off input processing */
  cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
			    |ICRNL |IXON|IXANY|IXOFF|IMAXBEL);
  /* pas IGNCR sinon il vire les 0x0D */
  cur_termios.c_iflag |= BRKINT;

  /* output_flags - turn off output processing */
  cur_termios.c_oflag  &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);

  /* control modes */
  cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL);
  cur_termios.c_cflag |= CREAD|CS8|CLOCAL;
  cur_termios.c_cflag &= ~(CRTSCTS);

  /* local modes */
  cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
  cur_termios.c_lflag |= NOFLSH;

  if (cfsetspeed(&cur_termios, br)) printf("setting modem serial device speed");

  if (tcsetattr(fd, TCSADRAIN, &cur_termios)) printf("setting modem serial device attr");

  return (fd);
}



struct pprz_servo_msg_struct
{
	//Data
	short s1; // 
	short s2; // 
	short s3; // 
	short s4; // 
	short s5; // 
	short s6; // 
};

void ubxSend(unsigned int cls, unsigned int id, struct pprz_servo_msg_struct * s);

static double gps_pos_x;
static double gps_pos_y;
static double gps_alt;
static double home_alt;
static double ant_azim = 180.;
static double ant_elev = 90.;
static int mode;
static int home_found;
static int ant_tracker_pan_mode = 180;
static double theta_servo_pw_span = 1000.;
static double psi_servo_pw_span = 1000.;
static double theta_servo_center_pw = 1500;
static double psi_servo_center_pw = 1500;
static char pololu_board_id = 12;
static char servo_acceleration = 3;
static char psi_servo_address = 1;
static char theta_servo_address = 0;

volatile int serial_error = 0;

double hfov = 180., vfov = 180.;
double hnp = 0., vnp = 0.;

unsigned char  speed = 0x00;

void set_servos(void);

GtkWidget *azim_scale;
GtkWidget *elev_scale;

void on_mode_changed(GtkRadioButton *radiobutton, gpointer user_data) {

  mode = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton)) ? MANUAL : AUTO;

}

void on_azimuth_changed(GtkAdjustment *hscale, gpointer user_data) {

if (mode == MANUAL) {
   ant_azim = gtk_range_get_value(GTK_RANGE (azim_scale));
   ant_elev = gtk_range_get_value(GTK_RANGE (elev_scale));
   //set_servos();
   }
}

//void on_elevation_changed(GtkRange *elev_scale, gpointer user_data) {
void on_elevation_changed(GtkAdjustment *hscale, gpointer user_data) {

if (mode == MANUAL) {
   ant_azim = gtk_range_get_value(GTK_RANGE (azim_scale));
   ant_elev = gtk_range_get_value(GTK_RANGE (elev_scale));
   //set_servos();
 }
}


#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
	gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)

GtkWidget* build_gui(void) {
	GtkWidget *window1;
	GtkWidget *vbox1;
	GtkWidget *vbox2;
	GtkWidget *table1;
	GtkWidget *label1;
	GtkWidget *label2;
	GtkWidget *label3;
	GtkWidget *label4;
	GtkWidget *radiobutton1;
	GSList *radiobutton1_group = NULL;
	GtkWidget *radiobutton2;
	GtkWidget *entry1;

	window1 = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW (window1), "tracking antenna");

	vbox1 = gtk_vbox_new(FALSE, 0);
	gtk_widget_show(vbox1);
	gtk_container_add(GTK_CONTAINER (window1), vbox1);

	vbox2 = gtk_vbox_new(FALSE, 0);
	gtk_widget_show(vbox2);
	gtk_box_pack_start(GTK_BOX (vbox1), vbox2, TRUE, TRUE, 0);

	table1 = gtk_table_new(4, 3, FALSE);
	gtk_widget_show(table1);
	gtk_box_pack_start(GTK_BOX (vbox2), table1, TRUE, TRUE, 0);
	gtk_table_set_col_spacings(GTK_TABLE (table1), 5);

	label1 = gtk_label_new("Azimuth");
	gtk_widget_show(label1);
	gtk_table_attach(GTK_TABLE (table1), label1, 0, 1, 1, 2,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label1), 0, 0.5);

	label2 = gtk_label_new("Elevation");
	gtk_widget_show(label2);
	gtk_table_attach(GTK_TABLE (table1), label2, 0, 1, 2, 3,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label2), 0, 0.5);

	label3 = gtk_label_new("Id");
	gtk_widget_show(label3);
	gtk_table_attach(GTK_TABLE (table1), label3, 0, 1, 3, 4,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label3), 0, 0.5);

	label4 = gtk_label_new("mode");
	gtk_widget_show(label4);
	gtk_table_attach(GTK_TABLE (table1), label4, 0, 1, 0, 1,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label4), 0, 0.5);

	radiobutton1 = gtk_radio_button_new_with_mnemonic(NULL, "manual");
	gtk_widget_show(radiobutton1);
	gtk_table_attach(GTK_TABLE (table1), radiobutton1, 1, 2, 0, 1,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_radio_button_set_group(GTK_RADIO_BUTTON (radiobutton1),
			radiobutton1_group);
	radiobutton1_group = gtk_radio_button_get_group(
			GTK_RADIO_BUTTON (radiobutton1));

	radiobutton2 = gtk_radio_button_new_with_mnemonic(NULL, "tracking");
	gtk_widget_show(radiobutton2);
	gtk_table_attach(GTK_TABLE (table1), radiobutton2, 2, 3, 0, 1,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_radio_button_set_group(GTK_RADIO_BUTTON (radiobutton2),
			radiobutton1_group);
	radiobutton1_group = gtk_radio_button_get_group(
			GTK_RADIO_BUTTON (radiobutton2));

	azim_scale = gtk_hscale_new(
			//GTK_ADJUSTMENT (gtk_adjustment_new (180, 0, 360, 1, 1, 1)));
			GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 360, 1, 1, 1)));
	gtk_widget_show(azim_scale);
	gtk_table_attach(GTK_TABLE (table1), azim_scale, 1, 3, 1, 2,
			(GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
			(GtkAttachOptions) (GTK_FILL), 0, 0);
//	gtk_range_set_update_policy(GTK_RANGE (azim_scale), GTK_UPDATE_DELAYED);

	elev_scale = gtk_hscale_new(
			//GTK_ADJUSTMENT (gtk_adjustment_new (45, 0, 90, 1, 1, 1)));
			GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 90, 1, 1, 1)));
	gtk_widget_show(elev_scale);
	gtk_table_attach(GTK_TABLE (table1), elev_scale, 1, 3, 2, 3,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (GTK_FILL), 0, 0);
//        gtk_range_set_update_policy(GTK_RANGE (elev_scale), GTK_UPDATE_DELAYED);

	entry1 = gtk_entry_new();
	gtk_widget_show(entry1);
	gtk_table_attach(GTK_TABLE (table1), entry1, 1, 3, 3, 4,
			(GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) (0),
			0, 0);

	g_signal_connect ((gpointer) radiobutton1, "toggled",
			G_CALLBACK (on_mode_changed),
			NULL);

	g_signal_connect ((gpointer) azim_scale, "value-changed",
			G_CALLBACK (on_azimuth_changed),
			NULL);

	g_signal_connect ((gpointer) elev_scale, "value-changed",
			G_CALLBACK (on_elevation_changed),
			NULL);

	/* Store pointers to all widgets, for use by lookup_widget(). */
	GLADE_HOOKUP_OBJECT_NO_REF (window1, window1, "window1");
	GLADE_HOOKUP_OBJECT (window1, vbox1, "vbox1");
	GLADE_HOOKUP_OBJECT (window1, vbox2, "vbox2");
	GLADE_HOOKUP_OBJECT (window1, table1, "table1");
	GLADE_HOOKUP_OBJECT (window1, label1, "label1");
	GLADE_HOOKUP_OBJECT (window1, label2, "label2");
	GLADE_HOOKUP_OBJECT (window1, label3, "label3");
	GLADE_HOOKUP_OBJECT (window1, label4, "label4");
	GLADE_HOOKUP_OBJECT (window1, radiobutton1, "radiobutton1");
	GLADE_HOOKUP_OBJECT (window1, radiobutton2, "radiobutton2");
	GLADE_HOOKUP_OBJECT (window1, entry1, "entry1");

	return window1;
}

void set_servos(void)
{

        double hpos, vpos;
        int hservo = theta_servo_center_pw, vservo = psi_servo_center_pw;

	// The magic is done here

		if (ant_tracker_pan_mode == 180){

	   // Take the vertical angle relative to the neutral point "vnp"
	   vpos = ant_elev - vnp;

		   // keep within the field of view "vfov"
		   if (vpos > (vfov / 2)) { vpos = vfov / 2; } else if(-vpos > (vfov / 2)){ vpos = -vfov / 2; }

		   // First take the horizontal angle relative to the neutral point "hnp"
	   hpos = ant_azim - hnp;

		   // Keep the range between (-180,180). this is done so that it consistently swaps sides
	   if (hpos < -180){ hpos += 360; }else if(hpos > 180){ hpos -= 360; }

		   // Swap sides to obtain 360 degrees of Azimuth coverage.
		   if(hpos > 90){ hpos = hpos-180;  vpos = 180-vpos; }else if(hpos < -90){ hpos = hpos+180;  vpos = 180-vpos; }

		   // keep the range within the field of view "hfov"
		   if (hpos > (hfov / 2)) { hpos = hfov / 2; } else if (-hpos > (hfov / 2)) { hpos = -hfov / 2;	}

		   // Convert angles to servo microsecond values suitable for the Pololu micro Maestro servo controller.
		   vpos = (psi_servo_center_pw-(psi_servo_pw_span/2)) + (vpos*(psi_servo_pw_span/vfov));
		   hpos = theta_servo_center_pw + (hpos*(theta_servo_pw_span/(hfov/2)));

		   //convert the values to integer.
		   hservo = hpos;
		   vservo = vpos;


		}else{
				vpos = ant_elev;

			// First take the horizontal angle relative to the neutral point "hnp"
			hpos = ant_azim - hnp;

				// Keep the range between (-180,180).
			if (hpos < -180){ hpos += 360; }else if(hpos > 180){ hpos -= 360; }

				// Keep the range between 0 to 360.
			//if (hpos < 0) { hpos += 360; } else if (hpos > 360){ hpos -= 360; }

				// keep the range within the field of view "hfov"
				if (hpos > (hfov / 2)) { hpos = hfov / 2; } else if (-hpos > (hfov / 2)) { hpos = -hfov / 2;	}

				// Convert angles to servo microsecond values suitable for the Pololu micro Maestro servo controller.
				vpos = (psi_servo_center_pw-(psi_servo_pw_span/2)) + (fabs(vpos)*(psi_servo_pw_span/vfov));
				hpos = theta_servo_center_pw + (hpos*(theta_servo_pw_span/hfov));

				//convert the values to integer.
				hservo = hpos;
				vservo = vpos;
			}

		   // Sanity check.
		   if (vservo < (psi_servo_center_pw-fabs(psi_servo_pw_span/2)) ){
			  vservo = (psi_servo_center_pw-fabs(psi_servo_pw_span/2));

		   }else if(vservo > (psi_servo_center_pw+fabs(psi_servo_pw_span/2))){ vservo = (psi_servo_center_pw+fabs(psi_servo_pw_span/2)); }

		   if (hservo < (theta_servo_center_pw-fabs(theta_servo_pw_span/2)) ){
			  hservo = (theta_servo_center_pw-fabs(theta_servo_pw_span/2));

		   }else if(hservo > (theta_servo_center_pw+fabs(theta_servo_pw_span/2))){ hservo = (theta_servo_center_pw+fabs(theta_servo_pw_span/2)); }

		   if (ant_azim > 180)
		     ant_azim -= 360;

                   if (ant_azim < -180)
                     ant_azim += 360;

    hservo = ant_azim / 180 * 8196;  //The pololu Maestro uses 0.25 microsecond increments so we need to multiply microseconds by 4.
		vservo = (ant_elev -40) / 50 * 8196; //The pololu Maestro uses 0.25 microsecond increments so we need to multiply microseconds by 4.

	g_message("vservo %i hservo %i", (int)(vservo), (int)(hservo)); //Divide by 4 so we can have the servo PW with 1 microsecond resolution.

	struct pprz_servo_msg_struct servos = { 0,hservo,vservo,0,0,0 };
	ubxSend(100, 5, &servos);
return;
}



/* jump here when a GPS message is received */
void on_GPS_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
	if (home_found == 0) {
		if (atof(argv[0]) == 3) { /* wait until we have a valid GPS fix */
			home_alt = atof(argv[4]) / 100.; /* get the altitude */
			home_found = 1;
		}
	}
	gps_alt = atof(argv[4]) / 100.;
}

/* jump here when a NAVIGATION message is received */
void on_NAV_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]) {


	if (mode == AUTO) {
		gps_pos_x = atof(argv[2]);
		gps_pos_y = atof(argv[3]);
		/* calculate azimuth */
				//should be "atan2(gps_pos_y, gps_pos_x)" but it is reversed to give 0 when North.
		ant_azim = atan2(gps_pos_x, gps_pos_y) * 180. / M_PI;

		if (ant_azim < 0)
			ant_azim += 360.;

		/* calculate elevation */
		ant_elev = atan2((gps_alt - home_alt), sqrt(atof(argv[5]))) * 180. / M_PI;
			// Sanity check
				if (ant_elev < 0){ ant_elev = 0.; }
		gtk_range_set_value(GTK_RANGE (azim_scale), ant_azim);
		gtk_range_set_value(GTK_RANGE (elev_scale), ant_elev);

				//set_servos();

	   }
}

/* jump here when a REMOTE_ANT message is received */
void on_REMOTE_ANT(IvyClientPtr app, void *user_data, int argc, char *argv[]) {


	if (mode == AUTO) {
		printf("Tes %f %f %f\n", atof(argv[1]), atof(argv[2]), atof(argv[3]));
		gps_pos_x = atof(argv[1]) - 3.;
		gps_pos_y = atof(argv[2]) - 3.4;
		gps_alt = atof(argv[3]);
		home_alt = 0.75;

		/* calculate azimuth */
				//should be "atan2(gps_pos_y, gps_pos_x)" but it is reversed to give 0 when North.
		ant_azim = atan2(gps_pos_x, gps_pos_y) * 180. / M_PI;

		if (ant_azim < 0)
			ant_azim += 360.;

		/* calculate elevation */
		ant_elev = atan2((gps_alt - home_alt), sqrt(pow(gps_pos_x, 2) + pow(gps_pos_y, 2))) * 180. / M_PI;
			// Sanity check
				if (ant_elev < 0){ ant_elev = 0.; }
		gtk_range_set_value(GTK_RANGE (azim_scale), ant_azim);
		gtk_range_set_value(GTK_RANGE (elev_scale), ant_elev);

				//set_servos();

	   }


}

int open_port(char* port ) {
	struct termios options;

	// would probably be good to set the port up as an arg.
	// The Pololu micro maestro registers two ports /dev/ttyACM0 and /dev/ttyACM1, /dev/ttyACM0 is the data port.
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		//perror("open_port: Unable to open /dev/ttyUSB1");
	        printf ("open_port: Unable to open %s \n", port);
	        serial_error = fd;
	} else
		printf("Success %s %s \n", port, "opened");
		fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);

	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);

	options.c_cflag |= (CLOCAL | CREAD);

	return (fd);
}

gboolean timeout_transmit_callback(gpointer data);
gboolean timeout_transmit_callback(gpointer data) {
	set_servos();
	return TRUE;
}

int main(int argc, char** argv) {

	gtk_init(&argc, &argv);

        int x = 0, y = 0, z = 0;
        char buffer[20];
        char serial_open = 0;
        printf ("Antenna Tracker for the Paparazzi autopilot, Chris Efstathiou 2010 \n");

        if(serial_open == 0){ printf ("Trying to open /dev/ttyUSB0 \n");  c_init_serial(); }

	GtkWidget* window = build_gui();
	gtk_widget_show_all(window);

	if (mode == MANUAL) {
	   ant_azim = gtk_range_get_value(GTK_RANGE (azim_scale));
	   ant_elev = gtk_range_get_value(GTK_RANGE (elev_scale));
	   //set_servos();
	}

	//struct pprz_servo_msg_struct servos = { 0,0,0,0,0,0 };
  //      ubxSend(100, 5, &servos);

  g_timeout_add(250, timeout_transmit_callback, NULL);

	IvyInit("AntennaTracker", "AntennaTracker READY", NULL, NULL, NULL, NULL);
	IvyBindMsg(
			on_GPS_STATUS,
			NULL,
			"^\\S* GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
	IvyBindMsg(on_NAV_STATUS, NULL,
			"^\\S* NAVIGATION (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
	IvyBindMsg(on_REMOTE_ANT, NULL,
			"^\\S* REMOTE_TRACK (\\S*) (\\S*) (\\S*) (\\S*)");
	IvyStart("127.255.255.255");
	gtk_main();

	return 0;
}


void ubxSend(unsigned int cls, unsigned int id, struct pprz_servo_msg_struct * s) {

	if (s == 0 || fd == NULL)
		return;

	//unsigned int msgSize = ((unsigned int) mysize + 8);

	unsigned short length = 12;

	if (length < SERIAL_BUFFER_SIZE)
	{
		unsigned char h[SERIAL_BUFFER_SIZE] =  { 0xb5, 0x62, (unsigned char) cls, (unsigned char) id };
		*((short*) &h[4]) = sizeof(struct pprz_servo_msg_struct);

		memcpy(h+6,(char*)s,sizeof(struct pprz_servo_msg_struct));

		unsigned char CK_A = 0;
                unsigned char CK_B = 0;
		int i;
		for(i=2;i<(sizeof(struct pprz_servo_msg_struct)+6);i++)
		{
			CK_A += h[i];
			CK_B += CK_A;
		}
		h[sizeof(struct pprz_servo_msg_struct)+6] = CK_A;
		h[sizeof(struct pprz_servo_msg_struct)+7] = CK_B;


		for (i=0;i<20;i++)
		  printf("%X ", h[i]);
                printf("\n");

		write(fd, (char*)h, sizeof(struct pprz_servo_msg_struct)+8);
		read(fd, (char*)h, 20);

		for (i=0;i<20;i++)
		  printf("%X ", h[i]);
                printf("\n");


    static int count = 0;
    if(h[7] == 0xA) {
    	count++;
    	if(count == 2)
    		pause();
    }
		return;
	}
	else
	{
		printf("Object larger than buffer\n");
		return;
	}
}