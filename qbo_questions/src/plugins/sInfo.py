#!/usr/bin/env python
#
#Copyright (C) 2012-2013 Thecorpora Inc.
#
#This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
#
#This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

import rospy

from qbo_system_info.srv import AskInfo

def hour(sentence,language):
    rospy.wait_for_service("/pluginsystem");
    service_pluginsystem = rospy.ServiceProxy('/pluginsystem', AskInfo)
    info = service_pluginsystem("hour")
    rospy.loginfo(info.info)
    return info.info

def date(sentence,language):
    rospy.wait_for_service("/pluginsystem");
    service_pluginsystem = rospy.ServiceProxy('/pluginsystem', AskInfo)
    info = service_pluginsystem("hdate")
    rospy.loginfo(info.info)
    return info.info
