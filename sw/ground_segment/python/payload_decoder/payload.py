#!/usr/bin/env python

import wx
import payload_decoder

class PayloadFrame(wx.App):
    def OnInit(self):
        self.main = payload_decoder.PayloadDecoderFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = PayloadFrame(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
