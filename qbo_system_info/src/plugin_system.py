#!/usr/bin/env python
#
#Copyright (C) 2012 Thecorpora SL
#
#This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
#
#This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

import sys
import ConfigParser
import commands
import os
import roslib; roslib.load_manifest('qbo_system_info')
import rospy
from qbo_system_info.srv import AskInfo


def handle_service(req):
    config=ConfigParser.ConfigParser()

    path = roslib.packages.get_pkg_dir("qbo_system_info")
    config.readfp(open(path+"/config/main.conf"))
    allsections=config.sections()
    for i in allsections:
	if req.command==config.get(i,"input"):
		execparams=config.get(i, "command")
		result=commands.getoutput(execparams)
		return result
    return "Error, input not exist"

def init_server():
    rospy.init_node('pluginsystem')
    s = rospy.Service('/pluginsystem', AskInfo, handle_service)
    rospy.spin()

if __name__ == "__main__":
    init_server()

