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

from __future__ import print_function
import socket
import telnetlib
import os
import sys
from ftplib import FTP
from time import sleep
import ftplib
import argparse
import re

class ParrotUtils:

    # Programs that could be running on the drone
    runnable_programs = [
        ('program.elf', 'Parrot native UAV application'),
        ('dragon-prog', 'Parrot native UAV application'),
        ('ap.elf', 'Paparazzi'),
        ('gst-launch', 'GStreamer')
    ]

    # Default values
    version_file = '/update/version.txt'
    upload_path = '/data/video/'
    uav_name = 'Parrot UAV'
    address = '192.168.1.1'

    # Initialize defaults
    def __init__(self):
        self.config_content = ''
        self.init_parser()

    # Connect with telnet and ftp, wait until login
    def connect(self):
        try:
            self.tn = telnetlib.Telnet(self.address, timeout=3)
            self.ftp = FTP(self.address)
            self.ftp.login()
            self.tn.expect(['\$ ', '# '], 10)
            return True
        except:
            print('#error Could not connect to the ' + self.uav_name + ' (address: ' + self.address + ')')
            print('Check if the ' + self.uav_name + ' is turned on and the computer is connected over wifi or bluetooth.')
            return False

    # Close the telnet and ftp
    def disconnect(self):
        self.tn.close()
        self.ftp.close()

    # Helper function
    def split_into_path_and_file(self, name):
        if name.count('/') <= 0:
            return ["./", name]
        return name.rsplit('/', 1)

    # Execute a command
    def execute_command(self, command):
        self.tn.write(command + '\n')
        s = self.tn.expect(['\$ ','# '], 10)[2]
        if s.endswith('[JS] $ '):
            s = s[len(command) + 2:-8]
        else:
            s = s[len(command) + 2:-4]
        return s

    # Upload ftp and catch memory-full error
    def upload(self, filename, content):
        try:
            self.ftp.storbinary("STOR " + filename, content)
        except ftplib.error_temp:
            print('#error Uploading the file to the ' + self.uav_name + ' failed!')
            print('Check if the Filesystem of the ' + self.uav_name + ' isn\'t full:')
            print(self.check_filesystem())
            sys.exit()
        except:
            print('#error Uploading the file to the ' + self.uav_name + ' failed!')
            print('FTP uploading failed with the following error: ', sys.exc_info()[0])
            print('Check if the Filesystem of the ' + self.uav_name + ' isn\'t full:')
            print(self.check_filesystem())
            sys.exit()

    # Download a file from the drown
    def download(self, filename, to_file):
        # Open file and download
        try:
            fd = open(to_file, 'wb')
            self.ftp.retrbinary('RETR ' + filename, fd.write)
            print('#pragma message: Download of "' + filename + '" from the ' + self.uav_name + ' success!')
        except IOError:
            print('#error Failed to open local file "' + to_file + '"')
        except:
            os.remove(to_file)
            print('#error Download of "' + filename + '" from ' + self.uav_name + ' Failed!')
        else:
            fd.close()


    ###### Sub functions #######
    # Check what currently is running on the drone
    def check_running(self):
        ps_aux = self.execute_command('ps')
        running = ""

        # Go trough all programings
        for prog in self.runnable_programs:
            if prog[0] in ps_aux:
                running += ' '+prog[1]+' ('+prog[0]+')'

        # Don't print the first space
        return running[1:]

    # Check the filesystem
    def check_filesystem(self):
        return self.execute_command('df -h')

    # Get the version of the drone
    def check_version(self):
        return re.sub('\s+','', self.execute_command('cat ' + self.version_file))

    # Default status
    def status(self):
        print('======================== ' + self.uav_name + ' Status ========================')
        self.uav_status()

        print('\n======================== Filesystem Status ========================')
        print(self.check_filesystem())

    # Reboot the drone
    def reboot(self):
        self.execute_command('reboot')
        print('The ' + self.uav_name + ' is now rebooting')

    # Kill a running program
    def kill_program(self, name):
        self.execute_command('killall -9 ' + name)
        print('Program "' + name + '" is now killed')

    # Start a new program
    def start_program(self, name):
        self.execute_command('chmod 777 ' + name)
        self.execute_command(name + ' > /dev/null 2>&1 &')
        print('Program "' + name + '" is now started')

    # Create a new directory
    def create_directory(self, name):
        self.execute_command('mkdir -p ' + name)
        print('Created new directory "' + name + '"')

    # Remove a directory
    def remove_directory(self, name):
        self.sexecute_command('rm -r ' + name)
        print('Removed directory "' + name + '"')

    # Upload a new file
    def upload_file(self, name, folder):
        f = self.split_into_path_and_file(name)

        # First kill the running program
        self.kill_program(f[1])

        # Make the upload directory and upload the file
        self.create_directory(self.upload_path + folder)
        self.upload(folder + '/' + f[1], file(name, "rb"))
        print('Succesfully uploaded "' + name + '" to folder "' + folder + '"')

    # Upload and run a new program
    def upload_and_run(self, name, folder):
        f = self.split_into_path_and_file(name)

        # Upload the file
        self.upload_file(name, folder)

        # Make the file executable and execute it
        self.start_program(self.upload_path + folder + '/' + f[1])
        print('#pragma message: Succesfully started "' + f[1] + '"')

#####################################################################

    # Main argument parser setup
    def init_parser(self):
        self.parser = argparse.ArgumentParser(description=self.uav_name + ' python helper. Use ' + sys.argv[0] + ' -h for help')
        self.parser.add_argument('--host', metavar='HOST', default=self.address,
                            help='the ip address of ' + self.uav_name)
        self.subparsers = self.parser.add_subparsers(title='Command to execute', metavar='command', dest='command')

        # Add commands
        self.subparsers.add_parser('status', help='Request the status of the ' + self.uav_name)
        self.subparsers.add_parser('reboot', help='Reboot the ' + self.uav_name)

        ss = self.subparsers.add_parser('kill', help='Kill a program on the ' + self.uav_name)
        ss.add_argument('program', help='The program to kill')

        ss = self.subparsers.add_parser('start', help='Start a program on the ' + self.uav_name)
        ss.add_argument('program', help='The program to start (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('upload', help='Upload a file to the ' + self.uav_name)
        ss.add_argument('file', help='Filename')
        ss.add_argument('folder', help='Destination subfolder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('download', help='Download a file from the ' + self.uav_name)
        ss.add_argument('file', help='Remote filename (could include folder)')
        ss.add_argument('save_file', help='Destination file on local computer')

        ss = self.subparsers.add_parser('upload_and_run', help='Upload and run software (for instance the Paparazzi autopilot)')
        ss.add_argument('file', help='Filename of an executable')
        ss.add_argument('folder', help='Remote destination folder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('mkdir', help='Make a new directory on the ' + self.uav_name)
        ss.add_argument('folder', help='Remote subfolder (base folder is the ftp folder)')

        ss = self.subparsers.add_parser('rmdir', help='Remove a directory and all its files from the ' + self.uav_name)
        ss.add_argument('folder', help='Remote subfolder (base folder is the ftp folder)')


    # Main function to parse arguments
    def parse_args(self):
        args = self.parser.parse_args()

        # First connect to the drone
        args.address = args.host
        if self.connect() == False:
            return False

        # Parse the command line arguments
        if args.command == 'status':
            self.status()
        elif args.command == 'reboot':
            self.reboot()
        elif args.command == 'kill':
            self.kill_program(args.program)
        elif args.command == 'start':
            self.start_program(self.upload_path + args.program)
        elif args.command == 'upload':
            self.upload_file(args.file, args.folder)
        elif args.command == 'download':
            self.download(args.file, args.save_file)
        elif args.command == 'upload_and_run':
            self.upload_and_run(args.file, args.folder)
        elif args.command == 'mkdir':
            self.remove_directory(self.upload_path + args.folder)
        elif args.command == 'rmdir':
            self.remove_directory(self.upload_path + args.folder)
        else:
            self.disconnect()
            return False

        # Disconnect
        self.disconnect()
        return True
