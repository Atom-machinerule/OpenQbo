#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012-2013 Thecorpora, Inc.
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
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;

import threading
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web

import json

import roslib; roslib.load_manifest('qbo_face_tracking')
roslib.load_manifest('qbo_pymouth')

import rospy
from std_msgs.msg import String

from lib_qbo_pyarduqbo import qbo_control_client
from lib_qbo_talk_py import qbo_talk_client
from qbo_pymouth import mouth

from time import sleep

from os import environ

from random import *

import string

import time

import signal

import commands

from syscall import runCmd

import urllib
import urllib2_file
import urllib2
import sys, json
import os
import tarfile

import subprocess


### Tornado security issues  ###
class BaseHandler(tornado.web.RequestHandler):
    def get_current_user(self):
        guestIp = repr(self.request.remote_ip)
        if guestIp == "'127.0.0.1'":
           user_id='local'
        else:
            user_id = self.get_secure_cookie("user")
        return user_id


class MainHandler(BaseHandler):
    @tornado.web.authenticated
    def get(self):
        name = tornado.escape.xhtml_escape(self.current_user)
        self.write("Hello, " + name)
        self.write("<br><br><a href=\"/auth/logout\">Log out</a>")


class AuthHandler(BaseHandler): 
    def get(self):
        self.write('<html><body><form action="/auth/login" method="post">'
                   '<table>'
                   '<tr><td>User name: </td><td><input type="text" name="username" id="username" /></td></tr>'
                   '<tr><td>Password: </td><td><input type="text" name="password" id="password" /></td></tr>'
                   '<tr><td></td><td><input type="submit" value="Submit"/></tr>'
                   '</table>'
                   '</form></body></html>')
    def post(self):
	# We give green light if the requests are comming from localhost
	'''
        guestIp = repr(self.request.remote_ip)
        print ">> "+guestIp
        if guestIp == "'127.0.0.1'":
                print 'Pass OKK'
		print ">>>>>>>>>>>>> "+	self.get_argument("username")	
		
		self.set_secure_cookie("user", self.get_argument("username"))
                print self.get_secure_cookie("user")

		print "ok go"
                self.redirect("/")
		print "ok go"
                return

	'''
        try:
            username = self.get_argument("username")
            password = self.get_argument("password")
        except:
            print 'User or Password incorrect'
            self.redirect("/")
            return

        #We open the file which has all the names and passwords
	path = roslib.packages.get_pkg_dir("qbo_http_api_login")
        f = open(path+"/config/users_pwd","r")
	try:
                # We look for the username and check if the password given matches
		for line in f:
			line = line.replace("\n","")
			parts = line.split(" ")
			if parts[0]==username:
				if parts[1] == password:
			                self.set_secure_cookie("user", self.get_argument("username")) 
        	        		self.redirect("/") 
					return
				else:
					print 'Authentication FAIL'
		        	        self.redirect("/auth/login")
					return
	
	
	except:			
		print 'Authentication FAIL'        		
		self.redirect("/auth/login")
		return

        #If we reach that far, is because the username given does not exist
	print 'Authentication FAIL'
        self.redirect("/auth/login")
        return


class LogoutHandler(BaseHandler): 
    def get(self): 
        self.clear_cookie("user") 
        self.redirect("/")

### End Tornado security issues ###




### Classes which are connected to resources ###
nodes_list={}

class qbo_mouth_web_api():
    def __init__(self):
        self.qbo_mouth_control=mouth()
        self.qbo_mouth_control_params=[]
        self.qbo_mouth_control_get_functions={}
        self.qbo_mouth_control_set_functions={}
        self.qbo_mouth_control_params.append('changeMouth')
        self.qbo_mouth_control_set_functions['changeMouth']=self.qbo_mouth_control.changeMouth

    def get(self,param):
        if param in self.qbo_mouth_control_get_functions.keys():
            return self.qbo_mouth_control_get_functions[param]()

    def put(self,param,data):
        if param in self.qbo_mouth_control_set_functions.keys():
            return self.qbo_mouth_control_set_functions[param](data['mouth'])

class qbo_talk_web_api():
    def __init__(self):
        self.qbo_talk_control=qbo_talk_client()
        self.qbo_talk_control_params=[]
        self.qbo_talk_control_get_functions={}
        self.qbo_talk_control_set_functions={}
        self.qbo_talk_control_params.append('say')
        self.qbo_talk_control_set_functions['say']=self.qbo_talk_control.say

    def get(self,param):
        if param in self.qbo_talk_control_get_functions.keys():
            return self.qbo_talk_control_get_functions[param]()

    def put(self,param,data):
        if param in self.qbo_talk_control_set_functions.keys():
            return self.qbo_talk_control_set_functions[param](data)

class qbo_face_traking_web_api():

    def __init__(self):
        self.face_traking_control_params=[]
        self.face_traking_control_get_functions={}
        self.face_traking_control_set_functions={}
        self.face_traking_control_params.append('start')
        self.face_traking_control_set_functions['start']=self.startFaceTraking
        self.face_traking_control_params.append('stop')
        self.face_traking_control_set_functions['stop']=self.stopFaceTraking
        self.started=False

    def startFaceTraking(self,data):
        print 'Face START'
        if not self.started:
            threading.Thread(target=runCmd,args=('rosrun face_tracking face_tracking',)).start()
            threading.Thread(target=runCmd,args=('rosrun face_following face_following',)).start()
            threading.Thread(target=runCmd,args=('roslaunch new_face_recognition face_recognition_android.launch',)).start()
            threading.Thread(target=runCmd,args=('roslaunch qbo_face_recognition qbo_face_recognition_demo.launch',)).start()
            self.started=True

    def stopFaceTraking(self,data):
        threading.Thread(target=runCmd,args=('rosnode kill /face_tracking_node',)).start()
        threading.Thread(target=runCmd,args=('rosnode kill /face_following_node',)).start()
        threading.Thread(target=runCmd,args=('rosnode kill /face_recognizer_node',)).start()
        threading.Thread(target=runCmd,args=('rosnode kill /face_recognizer_demo_node',)).start()
        self.started=False

    def get(self,param):
        if param in self.face_traking_control_get_functions.keys():
            return self.face_traking_control_get_functions[param]()

    def put(self,param,data):
        if param in self.face_traking_control_set_functions.keys():
            return self.face_traking_control_set_functions[param](data)

class qbo_stereo_web_api():
    def __init__(self):
        self.stereo_control_params=[]
        self.stereo_control_get_functions={}
        self.stereo_control_set_functions={}
        self.stereo_control_params.append('start')
        self.stereo_control_set_functions['start']=self.startStereo
        self.stereo_control_params.append('stop')
        self.stereo_control_set_functions['stop']=self.stopStereo
        self.started=False

    def startStereo(self,data):
        if not self.started:
            threading.Thread(target=runCmd,args=('rosrun stereo_anaglyph red_cyan_anaglyph.py __name:=stereo_anaglyph -c /stereo -d 20 -s',)).start()
            self.started=True

    def stopStereo(self,data):
        threading.Thread(target=runCmd,args=('rosnode kill /stereo_anaglyph',)).start()
        self.started=False

    def get(self,param):
        if param in self.stereo_control_get_functions.keys():
            return self.stereo_control_get_functions[param]()

    def put(self,param,data):
        if param in self.stereo_control_set_functions.keys():
            return self.stereo_control_set_functions[param](data)


class qbo_arduqbo_web_api():

    def __init__(self):
        self.qbo_control=qbo_control_client()
        self.qbo_arduqbo_params=[]
        self.qbo_arduqbo_get_functions={}
        self.qbo_arduqbo_set_functions={}
        #self.qbo_arduqbo_params.append('linearSpeed')
        #self.qbo_arduqbo_get_functions['linearSpeed']=self.qbo_control
        #self.qbo_arduqbo_set_functions['linearSpeed']=self.qbo_control
        #self.qbo_arduqbo_params.append('angularSpeed')
        #self.qbo_arduqbo_get_functions['angularSpeed']=self.qbo_control
        #self.qbo_arduqbo_set_functions['angularSpeed']=self.qbo_control
        self.qbo_arduqbo_params.append('speed')
        self.qbo_arduqbo_get_functions['speed']=self.qbo_control.speedGet
        self.qbo_arduqbo_set_functions['speed']=self.qbo_control.speedPut
        self.qbo_arduqbo_params.append('position')
        self.qbo_arduqbo_get_functions['position']=self.qbo_control.positionGet
        #self.qbo_arduqbo_params.append('robotPosX')
        #self.qbo_arduqbo_get_functions['robotPosX']=self.qbo_control
        #self.qbo_arduqbo_params.append('robotPosY')
        #self.qbo_arduqbo_get_functions['robotPosY']=self.qbo_control
        #self.qbo_arduqbo_params.append('robotPosTheta')
        #self.qbo_arduqbo_get_functions['robotPosTheta']=self.qbo_control
        #self.qbo_arduqbo_params.append('LCDstatus')
        #self.qbo_arduqbo_get_functions['LCDstatus']=self.qbo_control
        #self.qbo_arduqbo_params.append('LCDtext')
        #self.qbo_arduqbo_set_functions['LCDtext']=self.qbo_control
        self.qbo_arduqbo_params.append('LCD')
        #self.qbo_arduqbo_get_functions['LCD']=self.qbo_control
        self.qbo_arduqbo_set_functions['LCD']=self.qbo_control.LCDPut

        #self.qbo_arduqbo_params.append('SRFs')
        #self.qbo_arduqbo_get_functions['SRFs']=self.qbo_control.

        #self.qbo_arduqbo_params.append('SRF0')
        #self.qbo_arduqbo_get_functions['SRF']=self.qbo_control
        #self.qbo_arduqbo_params.append('SRF1')
        #self.qbo_arduqbo_get_functions['SRF1']=self.qbo_control
        #self.qbo_arduqbo_params.append('SRF2')
        #self.qbo_arduqbo_get_functions['SRF2']=self.qbo_control
        #self.qbo_arduqbo_params.append('SRF3')
        #self.qbo_arduqbo_get_functions['SRF3']=self.qbo_control
        #self.qbo_arduqbo_params.append('motorLeftState')
        #self.qbo_arduqbo_get_functions['motorLeftState']=self.qbo_control
        #self.qbo_arduqbo_set_functions['motorLeftState']=self.qbo_control
        #self.qbo_arduqbo_params.append('motorRightState')
        #self.qbo_arduqbo_get_functions['motorRightState']=self.qbo_control
        #self.qbo_arduqbo_set_functions['motorRightState']=self.qbo_control
        #self.qbo_arduqbo_params.append('motors')
        #self.qbo_arduqbo_get_functions['motors']=self.qbo_control
        #self.qbo_arduqbo_set_functions['motors']=self.qbo_control
        self.qbo_arduqbo_params.append('headServos')
        self.qbo_arduqbo_get_functions['headServos']=self.qbo_control.headServosGet
        self.qbo_arduqbo_set_functions['headServos']=self.qbo_control.headServosPut
        self.qbo_arduqbo_params.append('eyesServos')
        self.qbo_arduqbo_get_functions['eyesServos']=self.qbo_control.eyelidServosGet
        self.qbo_arduqbo_set_functions['eyesServos']=self.qbo_control.eyelidServosPut
        #self.qbo_arduqbo_params.append('panServoState')
        #self.qbo_arduqbo_get_functions['panServoState']=self.qbo_control
        #self.qbo_arduqbo_set_functions['panServoState']=self.qbo_control
        #self.qbo_arduqbo_params.append('tiltServoState')
        #self.qbo_arduqbo_get_functions['tiltServoState']=self.qbo_control
        #self.qbo_arduqbo_set_functions['tiltServoState']=self.qbo_control
        #self.qbo_arduqbo_params.append('panServoPosition')
        #self.qbo_arduqbo_get_functions['panServoPosition']=self.qbo_control
        #self.qbo_arduqbo_set_functions['panServoPosition']=self.qbo_control
        #self.qbo_arduqbo_params.append('tiltServoPosition')
        #self.qbo_arduqbo_get_functions['tiltServoPosition']=self.qbo_control
        #self.qbo_arduqbo_set_functions['tiltServoPosition']=self.qbo_control
        #self.qbo_arduqbo_params.append('leftEyelidServoPosition')
        #self.qbo_arduqbo_get_functions['leftEyelidServoPosition']=self.qbo_control
        #self.qbo_arduqbo_set_functions['leftEyelidServoPosition']=self.qbo_control
        #self.qbo_arduqbo_params.append('rightEyelidServoPosition')
        #self.qbo_arduqbo_get_functions['rightEyelidServoPosition']=self.qbo_control
        #self.qbo_arduqbo_set_functions['rightEyelidServoPosition']=self.qbo_control
        #self.qbo_arduqbo_params.append('panServoSpeed')
        #self.qbo_arduqbo_get_functions['panServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_set_functions['panServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_params.append('tiltServoSpeed')
        #self.qbo_arduqbo_get_functions['tiltServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_set_functions['tiltServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_params.append('leftEyelidServoSpeed')
        #self.qbo_arduqbo_get_functions['leftEyelidServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_set_functions['leftEyelidServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_params.append('rightEyelidServoSpeed')
        #self.qbo_arduqbo_get_functions['rightEyelidServoSpeed']=self.qbo_control
        #self.qbo_arduqbo_set_functions['rightEyelidServoSpeed']=self.qbo_control
        self.qbo_arduqbo_params.append('MICs')
        self.qbo_arduqbo_get_functions['MICs']=self.qbo_control.MICsGet
        #self.qbo_arduqbo_set_functions['MICs']=self.qbo_control.MICsPut
        #self.qbo_arduqbo_params.append('MIC0')
        #self.qbo_arduqbo_get_functions['MIC0']=self.qbo_control
        #self.qbo_arduqbo_params.append('MIC1')
        #self.qbo_arduqbo_get_functions['MIC1']=self.qbo_control
        #self.qbo_arduqbo_params.append('MIC2')
        #self.qbo_arduqbo_get_functions['MIC2']=self.qbo_control
        #self.qbo_arduqbo_params.append('micsMute')
        #self.qbo_arduqbo_set_functions['micsMute']=self.qbo_control
        self.qbo_arduqbo_params.append('battery')
        self.qbo_arduqbo_get_functions['battery']=self.qbo_control.batteryGet
        #self.qbo_arduqbo_params.append('batLevel')
        #self.qbo_arduqbo_get_functions['batLevel']=self.qbo_control

        self.qbo_arduqbo_params.append('test')
        self.qbo_arduqbo_get_functions['test']=self.qbo_control.testBoards
        #self.qbo_arduqbo_params.append('mouth')
        #self.qbo_arduqbo_set_functions['mouth']=self.qbo_control



    def get_node_params(self, node, params):
        if node=='qbo_arduqbo':
            ret_params={}
            for param in params:
                if param in self.qbo_arduqbo_params:
                  pass

    def get(self,param):
        if param in self.qbo_arduqbo_get_functions.keys():
            return self.qbo_arduqbo_get_functions[param]()

    def put(self,param,data):
        if param in self.qbo_arduqbo_set_functions.keys():
            return self.qbo_arduqbo_set_functions[param](data)


class qbo_sip_functions():
    def __init__(self):
        self.sip_control_params=[]
        self.sip_control_get_functions={}
        self.sip_control_set_functions={}
        self.sip_control_params.append('setIpSip')
        self.sip_control_set_functions['setIpSip']=self.setIpSip
        self.sip_control_params.append('getUserSipId')
        self.sip_control_get_functions['getUserSipId']=self.getUserSipId
        self.sip_control_params.append('getBotSipId')
        self.sip_control_get_functions['getBotSipId']=self.getBotSipId
        self.sip_control_params.append('endCall')
        self.sip_control_get_functions['endCall']=self.endCall
        self.sip_control_params.append('startSIPServer')
        self.sip_control_set_functions['startSIPServer']=self.startSIPServer
        self.sip_control_params.append('stopSIPServer')
        self.sip_control_get_functions['stopSIPServer']=self.stopSIPServer

        self.linphoneRunning = False
        self.auth = "notDefined"
        self.authBot = "notDefined"
        self.envi = ""

        self.processSipd = ""
        self.processSiprtmp = ""
        self.processLinphone = ""
        self.processAudioControl = ""

        self.ecoCancelationId = "notDefined"

    def startSIPServer(self,data):
        print "Start sip server"
        path2webi = roslib.packages.get_pkg_dir("qbo_webi")

        chars = string.ascii_letters + string.digits

        self.envi = environ.copy()
        path = self.envi["PYTHONPATH"]
        self.envi["PYTHONPATH"] = "/opt/ros/electric/stacks/qbo_stack/qbo_webi/src/teleoperation/sip2rtmp/p2p-sip:/opt/ros/electric/stacks/qbo_stack/qbo_webi/src/teleoperation/sip2rtmp/p2p-sip/src:/opt/ros/electric/stacks/qbo_stack/qbo_webi/src/teleoperation/sip2rtmp/p2p-sip/src/app:/opt/ros/electric/stacks/qbo_stack/qbo_webi/src/teleoperation/sip2rtmp/p2p-sip/src/external:/opt/ros/electric/stacks/qbo_stack/qbo_webi/src/teleoperation/sip2rtmp/rtmplite:"+path


        self.auth = "notDefined"
        self.authBot = "notDefined"

        self.auth = "".join(choice(chars) for x in range(randint(4, 4)))
        self.authBot = "".join(choice(chars) for x in range(randint(4, 4)))


        # We check if sipd is already active, if so, we close it        
        cmd = "ps -aux | grep sipd"
        out = runCmd(cmd)
        out = out[0]

        if "sipd.py" in out:
            pid = out.split(" ")[2]
            os.kill(int(pid), signal.SIGTERM)

        # The same with siprtmp
        cmd = "ps -aux | grep siprtmp"
        out = runCmd(cmd)
        out = out[0]

        if "siprtmp.py" in out:
            pid = out.split(" ")[2]
            os.kill(int(pid), signal.SIGTERM)  

        # Launch audio control with sip profile
        cmd = "roslaunch qbo_audio_control audio_control_sip.launch"
        self.processAudioControl = subprocess.Popen(cmd.split(),env=self.envi)

        # Launch sipd.py
        cmd = "python "+path2webi+"/src/teleoperation/sip2rtmp/p2p-sip/src/app/sipd.py -u "+self.auth+" -b "+self.authBot
        self.processSipd = subprocess.Popen(cmd.split(),env=self.envi)

        # Launch siprtmp.py
        cmd = "python "+path2webi+"/src/teleoperation/sip2rtmp/rtmplite/siprtmp.py"
        self.processSiprtmp = subprocess.Popen(cmd.split(),env=self.envi)

        # We give them sometime to finish the job
        time.sleep(0.5)

        # Data ready for the node qbo_linphone, but we still need to know the host
        rospy.set_param("linphone_botName",self.authBot)
        rospy.set_param("linphone_host","waiting for the mobile to know the IP")

        # ECO cancelation on
        if data['ecoCancelation']:
            cmd = "pactl load-module module-echo-cancel"
            out = runCmd(cmd);
            self.ecoCancelationId = out[0].replace("\n","")
            print "ECO cancelation ON "+str(self.ecoCancelationId)


    def stopSIPServer(self):
        try:
            self.processSiprtmp.send_signal(signal.SIGINT)
        except Exception as e:
            print "ERROR when killing a siprtmp. "+str(e)

        try:
            self.processLinphone.send_signal(signal.SIGINT)
        except Exception as e:
            print "ERROR when killing a linphone. "+str(e)

        try:
            self.processSipd.send_signal(signal.SIGINT)
        except Exception as e:
            print "ERROR when killing a sipd. "+str(e)

	# We go back to the default audio control
        cmd = "roslaunch qbo_audio_control audio_control_listener.launch"
        try:
            subprocess.Popen(cmd.split(),env=self.envi)
        except:
            print "ERROR when launching audio_control_listener.launch"+str(e)            

        # ECO cancelation off
        if self.ecoCancelationId != "notDefined":
            print "ECO Cancelation off "+str(self.ecoCancelationId)
            cmd = "pactl unload-module "+self.ecoCancelationId
            out = runCmd(cmd)
            #print "salida "+str(out)
            print "Done"

    def setIpSip(self,data):
        if self.linphoneRunning:
            try:
                self.processLinphone.send_signal(signal.SIGINT)
                self.linphoneRunning = False
            except Exception as e:
                print "ERROR when killing a proccess. "+str(e) 

            # We give them some time to finish the job
            sleep(1)

        rospy.set_param("linphone_host",data['ip'])

        # Now we know the IP, we can launch the linphone in the robot
        cmd = "roslaunch qbo_linphone launch_on_robot.launch"
        self.processLinphone = subprocess.Popen(cmd.split(),env=self.envi)

        rospy.wait_for_service('autocaller')

        self.linphoneRunning = True

    def getUserSipId(self):
        return self.auth

    def getBotSipId(self):
        return self.authBot

    def endCall(self):
	cmd = "linphonecsh hangup"
        subprocess.Popen(cmd.split(),env=self.envi)

    def get_info(self):
        pass

    def get(self,param):
        if param in self.sip_control_get_functions.keys():
            return self.sip_control_get_functions[param]()
           
    def put(self,param,data):
        if param in self.sip_control_set_functions.keys():
            return self.sip_control_set_functions[param](data)
### End of classes connected to resources ###



#
# Handler for GET and POST petitions for each node
# The URI has to be like /control/node/param
#
# -> "node" must be one of the names in the dictionary nodes_list. Is the name of the node in charge of the resource you are going to ask for
#
# -> "param" is the name of the resource you want to work with. It must be one of the names inside the dictionaries that each class has to have (usually are node_name_param, node_name_get_functions, node_name_set_functions)
#
# For example in order to know the robot speed, a GET petition is needed to /control/qbo_arduqbo/speed
#
# However if you want to modify it, then a POST to the same URI will do the job
#
class QboNodeHandler(BaseHandler):
    @tornado.web.authenticated
    def get(self,node,param): #Both node and param are strings
        return_value=''
        if node in nodes_list.keys():
            return_value=return_value+json.dumps(nodes_list[node].get(param))
        else:
            return_value=return_value+json.dumps(False)
        jsoncallback=self.get_argument("jsoncallback", None) #Function to be executed at the client side
        if jsoncallback:
            return_value=jsoncallback+'('+return_value+')'
        self.write(return_value)
        return

    @tornado.web.authenticated
    def post(self,node,param):
        return_value=''
        if node in nodes_list.keys():
            data=json.loads(self.get_argument("data", None)) #From json to object
            if not data:
                return_value=return_value+json.dumps(False)
            else:
                return_value=return_value+json.dumps(nodes_list[node].put(param,data))
        else:

            return_value=return_value+json.dumps(False)
        jsoncallback=self.get_argument("jsoncallback", None) #Function to be executed at the client side
        if jsoncallback:
            return_value=jsoncallback+'('+return_value+')'

        self.write(return_value)
        return



settings = {
    'auto_reload': True,
    'cookie_secret': '32oETzKXQAGaYdkL5gEmGeJJFuYh7EQnp2XdTP1o/Vo=',
    'login_url': '/auth/login',
}

application = tornado.web.Application([
    (r'/control/([0-9a-zA-Z_]+)/([0-9a-zA-Z_]+)', QboNodeHandler),
    (r'/', MainHandler),
    (r'/auth/login', AuthHandler),
    (r'/auth/logout', LogoutHandler),
], **settings)

def myspin():
    print 'Web Server ON'
    rospy.spin()
    if tornado.ioloop.IOLoop.instance().running():
        tornado.ioloop.IOLoop.instance().stop()

    nodes_list['sip'].stopSIPServer()
    
    exit()

if __name__ == "__main__":
        global nodes_list
        rospy.init_node('qbo_http_control')
        nodes_list['qbo_arduqbo']=qbo_arduqbo_web_api()
        nodes_list['face_traking']=qbo_face_traking_web_api()
        nodes_list['stereo_anaglyph']=qbo_stereo_web_api()
        nodes_list['qbo_talk']=qbo_talk_web_api()
        nodes_list['mouth']=qbo_mouth_web_api()
	nodes_list['sip'] = qbo_sip_functions()
        rospy.sleep(1)
        threading.Thread(target=myspin).start()
        http_server = tornado.httpserver.HTTPServer(application)
        http_server.listen(8880)
        tornado.ioloop.IOLoop.instance().start()



def runCmd(self,cmd, timeout=None):
    '''
    Will execute a command, read the output and return it back.
   
    @param cmd: command to execute
    @param timeout: process timeout in seconds
    @return: a tuple of three: first stdout, then stderr, then exit code
    @raise OSError: on missing command or if a timeout was reached
    '''

    ph_out = None # process output
    ph_err = None # stderr
    ph_ret = None # return code

    p = subprocess.Popen(cmd, shell=True,
                     stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE,
                     env=self.env)
    # if timeout is not set wait for process to complete
    if not timeout:
        ph_ret = p.wait()
    else:
        fin_time = time.time() + timeout
        while p.poll() == None and fin_time > time.time():
            time.sleep(1)

        # if timeout reached, raise an exception
        if fin_time < time.time():

            # starting 2.6 subprocess has a kill() method which is preferable
            # p.kill()
            os.kill(p.pid, signal.SIGKILL)
            raise OSError("Process timeout has been reached")

        ph_ret = p.returncode


    ph_out, ph_err = p.communicate()

    return (ph_out, ph_err, ph_ret)



