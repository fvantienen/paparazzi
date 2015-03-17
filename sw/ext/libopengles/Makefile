##
## Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
##
## This file is part of Paparazzi.
##
## Paparazzi is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2, or (at your option)
## any later version.
##
## Paparazzi is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with paparazzi; see the file COPYING.  If not, see
## <http://www.gnu.org/licenses/>.
##

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
endif

PREFIX	  ?= arm-none-eabi
CC		 		= $(PREFIX)-gcc
CFLAGS 		= -Wall
LDFLAGS   =


all: GLESv2.so EGL.so GLESv1_CM.so

%.so: %.o
	@printf "  CC -shared      $(<F)\n"
	$(Q)$(CC) -shared $(LDLAGS) -o $@ $<

%.o: %.c
	@printf "  CC      $(<F)\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

clean:
	$(Q)rm -f *.o *.d *.so

.PHONY: clean
