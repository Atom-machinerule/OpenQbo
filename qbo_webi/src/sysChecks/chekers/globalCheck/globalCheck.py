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
# Authors: Miguel Angel Julian <miguel.julian@openqbo.com>;
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
#          Arturo Bajuelos <arturo@openqbo.com>; 
#          Sergio Merino <s.merino@openqbo.com>;

import cherrypy
import os
from mako.template import Template
from sysChecks.chekers.chekers import chekers

import roslib
roslib.load_manifest('qbo_webi')
import rospy
from qbo_arduqbo.srv import Test

class globalCheck(chekers):

    def __init__(self,lang):
        chekers.__init__(self,'globalCheck',lang)

        #Codigo ROS
        self.test_client = rospy.ServiceProxy('/qbo_arduqbo/test_service', Test)

        self.htmlTmpl = Template(filename='sysChecks/chekers/'+self.name+'/templates/globalCheckTemplate.html')
        self.cssTmpl = Template(filename='sysChecks/chekers/'+self.name+'/templates/globalCheckTemplate.css')
        #print self.cssTmpl
        #print chekers.cssTmpl
        testDic=self.doTest()

    def unlockWheel(self,src):
        src=src[0]
        if (src=="left"):
            os.system("rosservice call /qbo_arduqbo/unlock_motors_stall")
        elif (src=="right"):
            os.system("rosservice call /qbo_arduqbo/unlock_motors_stall")
        return "ok"

    def get_html_content(self):
        testDic=self.doTest()
        return self.htmlTmpl.render(language=self.language, check_result=testDic)

    def doTest(self):
        try:
          testResponse = self.test_client()
          testDic={}
          testDic['SRFcount']=testResponse.SRFcount
          testDic['SRFAddress']=testResponse.SRFAddress
          testDic['SRFNotFound']=testResponse.SRFNotFound
          testDic['Gyroscope']=testResponse.Gyroscope
          testDic['Accelerometer']=testResponse.Accelerometer
          testDic['LCD']=testResponse.LCD
          testDic['Qboard3']=testResponse.Qboard3
          testDic['Qboard1']=testResponse.Qboard1
          testDic['Qboard2']=testResponse.Qboard2
          testDic['leftMotor']=testResponse.leftMotor
          testDic['rightMotor']=testResponse.rightMotor
          testDic['Total'] = testDic['Gyroscope'] and testDic['Accelerometer'] and testDic['LCD'] and testDic['Qboard3'] and testDic['Qboard2'] and testDic['Qboard1'] and testDic['leftMotor'] and testDic['rightMotor'] and len(testDic['SRFNotFound'])==0
          print 'Diccionario:',testDic
        #return self.htmlTmpl.render(language=self.language)
        except Exception, e:
          testDic={}
          testDic['SRFcount']=False
          testDic['SRFAddress']=False
          testDic['SRFNotFound']=False
          testDic['Gyroscope']=False
          testDic['Accelerometer']=False
          testDic['LCD']=False
          testDic['Qboard3']=False
          testDic['Qboard1']=False
          testDic['Qboard2']=False
          testDic['leftMotor']=False
          testDic['rightMotor']=False
          testDic['Total'] = False
          print 'Error: ',e
        if testDic['Total']:
            self.result='ok'
        else:
            self.result='fail'
        return testDic

