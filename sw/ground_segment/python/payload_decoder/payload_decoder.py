import wx
import sys
import os
import time
import threading
import math
import pynotify
import socket
import array
import Image
from cStringIO import StringIO

UDP_IP = "10.0.0.3"
UDP_PORT = 32000

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300


jpegheader = b'\xff\xd8\xff\xe0\x00\x10\x4a\x46\x49\x46\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xdb\x00\x43\x00\x20\x16\x18\x1c\x18\x14\x20\x1c\x1a\x1c\x24\x22\x20\x26\x30\x50\x34\x30\x2c\x2c\x30\x62\x46\x4a\x3a\x50\x74\x66\x7a\x78\x72\x66\x70\x6e\x80\x90\xb8\x9c\x80\x88\xae\x8a\x6e\x70\xa0\xda\xa2\xae\xbe\xc4\xce\xd0\xce\x7c\x9a\xe2\xf2\xe0\xc8\xf0\xb8\xca\xce\xc6\xff\xdb\x00\x43\x01\x22\x24\x24\x30\x2a\x30\x5e\x34\x34\x5e\xc6\x84\x70\x84\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xc6\xff\xc0\x00\x11\x08\x00\x64\x00\x64\x03\x01\x22\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x1f\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\xff\xc4\x00\xb5\x10\x00\x02\x01\x03\x03\x02\x04\x03\x05\x05\x04\x04\x00\x00\x01\x7d\x01\x02\x03\x00\x04\x11\x05\x12\x21\x31\x41\x06\x13\x51\x61\x07\x22\x71\x14\x32\x81\x91\xa1\x08\x23\x42\xb1\xc1\x15\x52\xd1\xf0\x24\x33\x62\x72\x82\x09\x0a\x16\x17\x18\x19\x1a\x25\x26\x27\x28\x29\x2a\x34\x35\x36\x37\x38\x39\x3a\x43\x44\x45\x46\x47\x48\x49\x4a\x53\x54\x55\x56\x57\x58\x59\x5a\x63\x64\x65\x66\x67\x68\x69\x6a\x73\x74\x75\x76\x77\x78\x79\x7a\x83\x84\x85\x86\x87\x88\x89\x8a\x92\x93\x94\x95\x96\x97\x98\x99\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xff\xc4\x00\x1f\x01\x00\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\xff\xc4\x00\xb5\x11\x00\x02\x01\x02\x04\x04\x03\x04\x07\x05\x04\x04\x00\x01\x02\x77\x00\x01\x02\x03\x11\x04\x05\x21\x31\x06\x12\x41\x51\x07\x61\x71\x13\x22\x32\x81\x08\x14\x42\x91\xa1\xb1\xc1\x09\x23\x33\x52\xf0\x15\x62\x72\xd1\x0a\x16\x24\x34\xe1\x25\xf1\x17\x18\x19\x1a\x26\x27\x28\x29\x2a\x35\x36\x37\x38\x39\x3a\x43\x44\x45\x46\x47\x48\x49\x4a\x53\x54\x55\x56\x57\x58\x59\x5a\x63\x64\x65\x66\x67\x68\x69\x6a\x73\x74\x75\x76\x77\x78\x79\x7a\x82\x83\x84\x85\x86\x87\x88\x89\x8a\x92\x93\x94\x95\x96\x97\x98\x99\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xff\xda\x00\x0c\x03\x01\x00\x02\x11\x03\x11\x00\x3f\x00'

class ThumbNailFromPayload:

    def add_payload(self, lst):
        if lst[1] == 255:
            self.thumb = self.pay
        elif lst[1] == 0:
            self.pay = lst[3:]
        else:
            self.pay.extend(lst[3:])


    def __init__(self):

        self.w = WIDTH
        self.pay = []
        self.thumb = []

    def get_image(self):

        self.jpgdata = jpegheader + bytearray(self.thumb)
        file_jpgdata = StringIO(self.jpgdata)
        img = Image.open(file_jpgdata)
        return img



class PayloadDecoderFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if msg.name == "PAYLOAD":
            pld = msg.get_field(0).split(",")
            b = []
            for p in pld:
                b.append(int(p))
            self.thumb.add_payload(b)
            self.data = b
            sock.sendto(bytearray(b), (UDP_IP, UDP_PORT))
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

    def GetRGB(self, x, y, bpp):
        # calculate some colour values for this sample based on x,y position
        r = g = b = 0
        if y < self.h/3:                           r = 255
        if y >= self.h/3 and y <= 2*self.h/3:      g = 255
        if y > 2*self.h/3:                         b = 255

        if bpp == 4:
            a = int(x * 255.0 / self.w)
            return r, g, b, a
        else:
            return r, g, b

    def MakeBitmapRGB(self, width, height):
        # Make a bitmap using an array of RGB bytes
        bpp = 3  # bytes per pixel
        bytes = array.array('B', [0] * width*height*bpp)

        for y in xrange(height):
            for x in xrange(width):
                offset = y*width*bpp + x*bpp
                r,g,b = self.GetRGB(x, y, bpp)
                bytes[offset + 0] = r
                bytes[offset + 1] = g
                bytes[offset + 2] = b

        self.rgbBmp = wx.BitmapFromBuffer(width, height, bytes)


    def OnPaint(self, e):
        tdx = -5
        tdy = -7

        w = self.w
        h = self.w

        self.MakeBitmapRGB(w,h)

        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        #dc.DrawCircle(w/2,w/2,w/2-1)
        font = wx.Font(11, wx.ROMAN, wx.BOLD, wx.NORMAL)
        dc.SetFont(font)
        #dc.DrawText("PAYLOAD",2,2)
        dc.DrawText(str(self.data),2,2)

        myPilImage = self.thumb.get_image()
        myWxImage = wx.EmptyImage( myPilImage.size[0], myPilImage.size[1] )
        myWxImage.SetData( myPilImage.convert( 'RGB' ).tostring() )
        #print(myPilImage,myWxImage)
        bitmap = wx.BitmapFromImage(myWxImage)
        dc.DrawBitmap(bitmap, 30,30, True)

        #c = wx.Colour(0,0,0)
        #dc.SetBrush(wx.Brush(c, wx.SOLID))
        #dc.DrawCircle(int(w/2),int(w/2),10)



    def __init__(self):

        self.thumb = ThumbNailFromPayload()
        self.data = [];

        self.w = WIDTH
        self.h = WIDTH

        wx.Frame.__init__(self, id=-1, parent=None, name=u'PayloadDecoder',
                          size=wx.Size(self.w, self.h), title=u'Payload Decoder')

        ico = wx.Icon(os.getenv("PAPARAZZI_HOME") + "/sw/ground_segment/python/payload_decoder/camera.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)


        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)


        self.interface = IvyMessagesInterface("PayloadDecoder")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
