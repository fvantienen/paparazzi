#!/usr/bin/env python

import sys
import os
import time
import threading
import math
from time import sleep
import httplib
import json


sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

UPDATE_INTERVAL = 2

class TrackingFrame(object):
    def message_recv(self, ac_id, msg):
        if msg.name == "FLIGHT_PARAM":
            self.data['acid'] = msg['ac_id'].replace('replay','')
            self.data['fix'] = 3
            self.data['alt'] = float(msg['alt'])
            self.data['lat'] = float(msg['lat'])
            self.data['lon'] = float(msg['long'])
            print(self.data)
        if msg.name == "ROTORCRAFT_STATUS":
        #    print(msg)
            self.data['inflight'] = int(msg['ap_in_flight']);
	

    def __init__(self, verbose):
        self.verbose = verbose
        self.data = {}

        self.interface = IvyMessagesInterface("radiowatchframe")
        self.interface.subscribe(self.message_recv)
        print("Listening to IVY")

    def shutdown(self):
        self.interface.shutdown()


class Uploader(object):
    def __init__(self, verbose=1):
        self.verbose = verbose

        self.headers = {
            "Content-type": "application/json",
            "Accept": "application/json",
            "Authorization": "basic paparazzi=="
            }

        self.conn = httplib.HTTPConnection("www.dronetracking.net")

    def shutdown(self):
        self.conn.close()


    def upload(self, data):

        jsondata=json.dumps(data)
        self.conn.request("POST", "/real_time_tracker.php", jsondata, self.headers)
        r1 = self.conn.getresponse()
	

        print("Response:", r1.status, r1.reason)
        data1 = r1.read().decode('utf8')  # This will return entire content.

        json_object = json.loads(data1)
        print(json_object)



if __name__ == '__main__':
    try:
        fl = TrackingFrame(verbose=1)
        up = Uploader()
        for i in range(0,10000):
            sleep(UPDATE_INTERVAL)
            print(fl.data)
            up.upload(fl.data)
    except KeyboardInterrupt:
        print("Stopping on request")
    fl.shutdown()
    up.shutdown()
