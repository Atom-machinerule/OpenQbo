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
import json
from qbo_internet_services.srv import InternetService

def location(sentence,language):
    rospy.wait_for_service("/internetservices");
    service_iservices = rospy.ServiceProxy('/internetservices', InternetService)
    info = service_iservices("location","")
    decodedData=json.loads(info.info)
    if type(decodedData) is dict:
        city=decodedData['city']
        country=decodedData['country']
    else:
        print "Not valid JSON recived"
    response="We are in "+city+", in "+country
    rospy.loginfo(response)
    return response

def weather(sentence,language):
    rospy.wait_for_service("/internetservices");
    service_iservices = rospy.ServiceProxy('/internetservices', InternetService)
    info = service_iservices("weather","")
    decodedData=json.loads(info.info)
    if type(decodedData) is dict:
        temp=decodedData['temp']

# Convert to celsius(comment farenheit conversion if you uncomment this line):
        temp=temp-272.15

# Convert to Farenheit (comment celsius conversion if you uncomment this line)
#        temp=((temp*9)/5)-459.67
        if temp-int(temp)>=0.5:
            temp=int(temp)+1
        else:
            temp=int(temp)
        desc=decodedData['description']
    else:
        print "Not valid JSON recived"
    response="Today is "+desc+" and the temperature is of "+str(temp) +" degrees"
    rospy.loginfo(response)
    return response

