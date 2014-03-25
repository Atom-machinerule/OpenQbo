#!/usr/bin/env python2.6
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012 TheCorpora SL
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
# Authors: Arturo Bajuelos <arturo@openqbo.com>; 
#          Sergio Merino <s.merino@openqbo.com>;
#          etc.
import cherrypy
from mako.template import Template
from tabsClass import TabClass
import rospy

from std_msgs.msg import String
from sensor_msgs.msg  import Image

import roslib.packages

import unicodedata

import time
import urllib
import urllib2_file
import urllib2
import sys, json
import os
import shutil

class TestManager(TabClass):


    def __init__(self,language):
        self.language = language
        self.htmlTemplate = Template(filename='test/templates/testTemplate.html')
        self.jsTemplate = Template(filename='test/templates/testTemplate.js')


    @cherrypy.expose
    def unload(self):
        #self.mjpegServer.stop("8081")
        return "ok"

    @cherrypy.expose
    def index(self):
        #self.mjpegServer.start("8081")
        return self.htmlTemplate.render(language=self.language)

    @cherrypy.expose
    def testJs(self, parameters=None):
        return self.jsTemplate.render(language=self.language)

