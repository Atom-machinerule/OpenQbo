#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012 OpenQbo, Inc.
#
# This program is free software; you can redistribute it and/or 
# modify it under the terms of the GNU General Public License as 
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program; if not, write to the Free Software 
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
# MA 02110-1301, USA.
#
# Authors: Miguel Angel Julian <miguel.a.j@openqbo.com>; 
#

import roslib; roslib.load_manifest('qbo_talk')
import rospy
from qbo_talk.srv import Text2Speach

class qbo_talk_client():
    def __init__(self):
        self.say_client = rospy.ServiceProxy('/qbo_talk/festival_say', Text2Speach)

    def say(self,data):
        try:
            resp1 = self.say_client(str(data['sentence']))
            return True
        except Exception, e:
            print 'Error: ',e
            return False
