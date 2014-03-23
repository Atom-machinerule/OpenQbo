#!/usr/bin/env python

""" 
  A simple Controller GUI to drive robots and pose heads.
  Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import roslib; roslib.load_manifest('qbo_arduqbo')
import rospy
import wx

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

width = 300

class controllerGUI(wx.Frame):
    TIMER_ID = 100

    def __init__(self, parent, debug = False):  
        wx.Frame.__init__(self, parent, -1, "QBO Controller GUI", style = wx.DEFAULT_FRAME_STYLE & ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
        sizer = wx.GridBagSizer(10,10)

        # Move Base
        drive = wx.StaticBox(self, -1, 'Move Base')
        drive.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        driveBox = wx.StaticBoxSizer(drive,orient=wx.VERTICAL) 
        self.movebase = wx.Panel(self,size=(width,width-20))
        self.movebase.SetBackgroundColour('WHITE')
        self.movebase.Bind(wx.EVT_MOTION, self.onMove)  
        wx.StaticLine(self.movebase, -1, (width/2, 0), (1,width), style=wx.LI_VERTICAL)
        wx.StaticLine(self.movebase, -1, (0, width/2), (width,1))
        driveBox.Add(self.movebase)        
        sizer.Add(driveBox,(0,0),wx.GBSpan(1,1),wx.EXPAND|wx.TOP|wx.RIGHT|wx.LEFT,5)
        self.forward = 0
        self.turn = 0
        self.X = 0
        self.Y = 0

        # Move Head
        head = wx.StaticBox(self, -1, 'Move Head')
        head.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        headBox = wx.StaticBoxSizer(head,orient=wx.VERTICAL) 
        headSizer = wx.GridBagSizer(5,5)
        headSizer.Add(wx.StaticText(self, -1, "Pan:"),(0,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        headSizer.Add(wx.StaticText(self, -1, "L"),(0,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.pan = wx.Slider(self, -1, 0, -157, 157, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL )
        headSizer.Add(self.pan,(0,2))
        headSizer.Add(wx.StaticText(self, -1, "R"),(0,3), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

        headSizer.Add(wx.StaticText(self, -1, "Tilt:"),(1,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        headSizer.Add(wx.StaticText(self, -1, "U"),(1,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.tilt = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL)
        headSizer.Add(self.tilt,(1,2))
        headSizer.Add(wx.StaticText(self, -1, "D"),(1,3), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

        headSizer.Add(wx.StaticText(self, -1, "Left Eye:"),(2,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        headSizer.Add(wx.StaticText(self, -1, "CCW"),(2,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.leftEye = wx.Slider(self, -1, 0, -180, 180, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL)
        headSizer.Add(self.leftEye,(2,2))
        headSizer.Add(wx.StaticText(self, -1, "CW"),(2,3), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

        headSizer.Add(wx.StaticText(self, -1, "Right Eye:"),(3,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        headSizer.Add(wx.StaticText(self, -1, "CW"),(3,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.rightEye = wx.Slider(self, -1, 0, -180, 180, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL)
        headSizer.Add(self.rightEye,(3,2))
        headSizer.Add(wx.StaticText(self, -1, "CCW"),(3,3), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

        headBox.Add(headSizer) 
        sizer.Add(headBox, (1,0), wx.GBSpan(1,1), wx.EXPAND|wx.BOTTOM|wx.RIGHT|wx.LEFT,5)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(50)
        wx.EVT_CLOSE(self, self.onClose)
        wx.EVT_TIMER(self, self.TIMER_ID, self.onTimer)

        # bind the panel to the paint event
        wx.EVT_PAINT(self, self.onPaint)
        self.dirty = 1
        self.onPaint()

        self.SetSizerAndFit(sizer)
        self.Show(True)

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist)
        self.cmd_joints = rospy.Publisher('cmd_joints', JointState)

    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def onPaint(self, event=None):
        # this is the wx drawing surface/canvas
        dc = wx.PaintDC(self.movebase)
        dc.Clear()
        # draw crosshairs
        dc.SetPen(wx.Pen("black",1))
        dc.DrawLine(width/2, 0, width/2, width)
        dc.DrawLine(0, width/2, width, width/2)
        dc.SetPen(wx.Pen("red",2))
        # draw arm        
        #dc.DrawLine(width/2, width/2, (width/2) + self.turn*(width/2), (width/2) - self.forward*(width/2))
        dc.SetBrush(wx.Brush('red', wx.SOLID))
        dc.SetPen(wx.Pen("black",2))
        dc.DrawCircle((width/2) + self.X*(width/2), (width/2) - self.Y*(width/2), 5)  

    def onMove(self, event=None):
        if event.LeftIsDown():        
            pt = event.GetPosition()
            if pt[0] > 0 and pt[0] < width and pt[1] > 0 and pt[1] < width:
                self.forward = ((width/2)-pt[1])/2
                self.turn = (pt[0]-(width/2))/2 
                self.X = (pt[0]-(width/2.0))/(width/2.0)
                self.Y = ((width/2.0)-pt[1])/(width/2.0)        
        else:
            self.forward = 0; self.Y = 0
            self.turn = 0; self.X = 0
        self.onPaint()          
        
    def onTimer(self, event=None):
        # send joint updates
        j = JointState()
        j.name = ["head_pan_joint", "head_tilt_joint", "left_eyelid_joint", "right_eyelid_joint"]
        j.position = [-self.pan.GetValue()/100.0, self.tilt.GetValue()/100.0, self.leftEye.GetValue()/100.0, self.rightEye.GetValue()/100.0]
        self.cmd_joints.publish(j)
        # send base updates
        t = Twist()
        t.linear.x = self.forward/200.0; t.linear.y = 0; t.linear.z = 0
        if self.forward > 0:
            t.angular.x = 0; t.angular.y = 0; t.angular.z = -self.turn/50.0
        else:
            t.angular.x = 0; t.angular.y = 0; t.angular.z = self.turn/50.0
        self.cmd_vel.publish(t)

if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('controllerGUI')
    app = wx.PySimpleApp()
    frame = controllerGUI(None, True)
    app.MainLoop()

