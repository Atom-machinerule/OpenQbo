#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012 Thecorpora, Inc.
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
# Authors: Daniel Cuadrado Sanchez <daniel.cuadrado@openqbo.com>; 


import roslib; roslib.load_manifest('qbo_say_ip')
import rospy
from std_msgs.msg import String
from qbo_listen.msg import Listened
from qbo_system_info.srv import AskInfo
from qbo_talk.srv import Text2Speach

#
# Once the user ask about the robot's IP, we call the pluginsystem service and extract the
# each IP for each possible connection (wlan, eth, etc).
#
# Input:
#        None
#            
# Output:
#        None
#
def sayIP():

  
    rospy.wait_for_service("/pluginsystem");
    
    service_pluginsystem = rospy.ServiceProxy('/pluginsystem', AskInfo)
    
    
    print "uoo uooo"
    info = service_pluginsystem("netconf")
        
    rospy.loginfo(" IP  "+str(info.info))
    ip = str(info.info)
    ip = ip.replace("."," dot ")
        
    ip = ip.replace("eth","ethernet number ")
    ip = ip.replace("wlan","wifi number ")
        
    lines = ip.split("\n")
    print "--"+str(lines)
    for i in range(0,len(lines)):
        words = lines[i].split(" ")
    	if len(words)>=4: # four because the minimun string is "wifi number #"
    	    sentence = str(words[0])+" "+str(words[1])+" "+str(words[2])+"."+str(words[3:])
    	    sentence = sentence.replace("[","")
            sentence = sentence.replace("]","")
            sentence = sentence.replace("'","")
            sentence = sentence.replace(",","")
            print "about to say "+ sentence
            say(sentence)

#
# This function makes the robot to say whatever is given as an input
#
# Input:
#        sentence:    the string you want to be said
#            
# Output:
#        None
#
def say(sentence):
    rospy.wait_for_service("/Qbo/festivalSay")
    festival = rospy.ServiceProxy("/qbo_talk/festival_say", Text2Speach  )
    
    festival(sentence)


def init():
    rospy.init_node('qbo_say_ip')    
    rospy.loginfo('QBO sayIP Node launched')    
    sayIP() 


if __name__== "__main__":
    try:
        init()
    except rospy.ROSInterruptException: pass


