import wx
import sys
import os
import time
import threading
import math
import pynotify
import socket

UDP_IP = "192.168.127.255"
UDP_PORT = 32000

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET, # Internet
             socket.SOCK_DGRAM) # UDP

sys.path.append(os.getenv("PAPARAZZI_HOME") + "/sw/ext/pprzlink/lib/v1.0/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 300


class PayloadDecoderFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if msg.name == "PAYLOAD":
            print('PAYLOAD', msg)
            sock.sendto(msg, (UDP_IP, UDP_PORT))
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

    def OnPaint(self, e):
        tdx = -5
        tdy = -7

        w = self.w
        h = self.w

        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        dc.DrawCircle(w/2,w/2,w/2-1)
        font = wx.Font(11, wx.ROMAN, wx.BOLD, wx.NORMAL)
        dc.SetFont(font)
        dc.DrawText("PAYLOAD",2,2)

        c = wx.Colour(0,0,0)
        dc.SetBrush(wx.Brush(c, wx.SOLID))
        dc.DrawCircle(int(w/2),int(w/2),10)



    def __init__(self):

        self.w = WIDTH
        self.h = WIDTH

        wx.Frame.__init__(self, id=-1, parent=None, name=u'PayloadDecoder',
                          size=wx.Size(self.w, self.h), title=u'Payload Decoder')
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.interface = IvyMessagesInterface("PayloadDecoder")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
