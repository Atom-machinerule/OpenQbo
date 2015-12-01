#!/usr/bin/env python
#
#Copyright (C) 2012-2013 Thecorpora SL
#
#This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
#
#This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

import roslib; roslib.load_manifest('qbo_internet_services')
import rospy
from qbo_internet_services.srv import InternetService

import json
from xml.dom import minidom
import httplib


def server_request(server,url="",method="GET"):
    connection = httplib.HTTPConnection(server)
    connection.request(method,url)
    resp=connection.getresponse()
    print "Server:"+server+"|method:"+method+"|url:"+url
    if resp.status==200:
        result=resp.read()
        print "Remote response:"+result
    else:
        print "Server:"+server+"|method:"+method+"|url:"+url
        print "Status:"+resp.status
        print "Reason:"+resp.reason
        result=-1
    return result

def geoip_Location():
    location=server_request("api.hostip.info") 
    XMLinfo=minidom.parseString(location)
    node=XMLinfo.getElementsByTagName("Hostip")

    cityXML=node[0].getElementsByTagName("gml:name")
    city=cityXML[0].firstChild.nodeValue
    countryXML=node[0].getElementsByTagName("countryName")
    country=countryXML[0].firstChild.nodeValue
    countryAbXML=node[0].getElementsByTagName("countryAbbrev")
    countryAb=countryAbXML[0].firstChild.nodeValue
    coordinatesXML=node[0].getElementsByTagName("gml:coordinates")
    coordinates=coordinatesXML[0].firstChild.nodeValue
    coordinates_split=coordinates.split(",");
    latitude=coordinates_split[1]
    longitude=coordinates_split[0]

    formated_loc=json.dumps({"city":city,"country":country,"countryAb":countryAb,"latitude":latitude,"longitude":longitude})

    return formated_loc


def weather(coordinates):

    if coordinates=="":
        coordinates=geoip_Location()
    try:
        coordinates=coordinates.replace("\\","")
        decodedCoor=json.loads(coordinates)
        if type(decodedCoor) is dict:
            latitude=decodedCoor['latitude']
            longitude=decodedCoor['longitude']
        else:
            print "Not valid JSON recived"
    except ValueError:
        print "Not valid JSON recived, can't be decode"
        return "-1"
    except KeyError:
        print "Latitude or longitude not present in parameters"
        return "-1"


    weatherJson=server_request("openweathermap.org","/data/2.1/find/city?lon="+longitude+"&lat="+latitude+"&cnt=1")
    print "LET's process weather"
    try:
        decodedW=json.loads(weatherJson)
    except ValueError:
        print "Not valid JSON recived from weather API"
        return "-1"

    try:
        if type(decodedW) is dict:
            allParams=decodedW['list'][0]


            mainParam=allParams["main"]
            generalDesc=allParams["weather"][0]
            wind=allParams["wind"]
            del generalDesc["id"]
            del generalDesc["icon"]

            response=dict(mainParam.items()+generalDesc.items()+wind.items())
            response=json.dumps(response)
            
        else:
            print "Not valid JSON recived from weather API"
    except KeyError:
        print "Problems in weather JSON"
        return "-1"
    return response



def handle_service(req):
    print "Service called:"+req.service
    print "Param1:"+req.params
    if req.service=="location":
        response="1"
        response=geoip_Location()
    elif req.service=="weather":
        response=weather(req.params)
    else:
        print "Service doesn't exist"
        response="-1"
    print "Processed response:"+response
    return response

def init_server():
    rospy.init_node('Internet_Services')
    s = rospy.Service('/internetservices', InternetService, handle_service)
    rospy.spin()

if __name__ == "__main__":
    init_server()
