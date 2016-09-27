#!/usr/bin/env python

import wx
import atc_frame

class Atc(wx.App):
    def OnInit(self):
        self.main = atc_frame.AtcFrame()
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = Atc(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
