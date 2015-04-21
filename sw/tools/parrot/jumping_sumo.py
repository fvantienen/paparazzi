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

class JumpingSumo(ParrotUtils):
    uav_name = 'Jumping Sumo'
    address = '192.168.2.1'
    version_file = '/version.txt'
    upload_path = '/data/ftp/'

    def uav_status(self):
        print('Parrot version:\t\t' + self.check_version())
        print('Host address:\t\t' + self.address)
        print('Currently running:\t' + self.check_running())


if __name__ == "__main__":
    js = JumpingSumo()
    js.parse_args()
    exit(0)
