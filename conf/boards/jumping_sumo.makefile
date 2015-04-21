# Hey Emacs, this is a -*- makefile -*-
#
# jumping_sumo.makefile
#
# http://wiki.paparazziuav.org/wiki/Bebop
#

BOARD=jumping_sumo
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.parrot (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = parrot

# -----------------------------------------------------------------------
USER=foobar
HOST?=192.168.2.1
SUB_DIR=internal_000/paparazzi
FTP_DIR=/data/ftp
TARGET_DIR=$(FTP_DIR)/$(SUB_DIR)
# -----------------------------------------------------------------------

# The datalink default uses UDP
MODEM_HOST         ?= 192.168.2.255

# handle linux signals by hand
$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED				?= none
BARO_LED           			?= none
AHRS_ALIGNER_LED   			?= 1
GPS_LED            			?= none
SYS_TIME_LED       			?= 0
