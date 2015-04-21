#!/usr/bin/env python
#
# Copyright (C) 2012-2014 The Paparazzi Team
#               2015 Freek van Tienen <freek.v.tienen@gmail.com>
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, see
# <http://www.gnu.org/licenses/>.
#

from parrot_utils import ParrotUtils
import re

class Ardrone2(ParrotUtils):
    uav_name = 'Ardrone 2'
    config_file = '/data/config.ini'

    # Read from config.ini
    def read_from_config(self, name):
        # Only read file once
        if self.config_content == '':
            self.config_content = self.execute_command('cat ' + self.config_file)

        # Search for the name
        search = re.search(name + '[^=]+=[\r\n\t ]([^\r\n\t ]+)',self.config_content)
        if search is None:
            return None
        else:
            return search.group(1)

    # Write to config
    def write_to_config(self, name, value):
        if read_from_config(name) == None:
            self.execute_command('echo "' + name + ' = ' + value + '\" >> ' + self.config_file)
        else:
            self.execute_command('sed -i "s/\(' + name + ' *= *\).*/\\1' + value + '/g" ' + self.config_file)

    def uav_status(self):
        print('Parrot version:\t\t' + self.check_version())
        print('Host address:\t\t' + self.address)
        print('Currently running:\t' + self.check_running())
        print('Serial number:\t\t' + self.read_from_config('drone_serial'))
        print('Network id:\t\t' + self.read_from_config('ssid_single_player'))


if __name__ == "__main__":
    ardrone2 = Ardrone2()
    ardrone2.parse_args()
    exit(0)
