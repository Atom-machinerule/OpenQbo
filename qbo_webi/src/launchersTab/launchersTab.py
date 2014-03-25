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
# Authors: Miguel Julian <miguel.julian@openqbo.com>; 

import cherrypy
from mako.template import Template
from tabsClass import TabClass
import rospy
import subprocess
from subprocess import Popen, PIPE, STDOUT
import signal
import shlex

class LaunchersTabManager(TabClass):

    def __init__(self,language):
        self.language = language
        self.htmlTemplate = Template(filename='launchersTab/templates/launchersTabTemplate.html')

    @cherrypy.expose
    def index(self):
        return self.htmlTemplate.render(language=self.language)

    @cherrypy.expose
    def unload(self):
        return "ok"

    @cherrypy.expose
    def stopAll(self):
        print "parando todo por si acaso"
        self.stopRandommove()
        self.stopFacetraking()
        self.stopPhonelauncher()
        self.stopRandommove()
        self.stopQuestions()
        return "ok"
    
    @cherrypy.expose
    def phonelauncher(self):
        try:
            cmd = "rosrun qbo_mjpeg_server mjpeg_server"
            self.mjpegNode = subprocess.Popen(cmd.split())
            cmd = "rosrun qbo_http_api_login qbo_http_api_login.py"
            self.httpapiNode = subprocess.Popen(cmd.split())
        except:
            return "false"
        return "true"

    @cherrypy.expose
    def phonelauncher(self):
        try:
            cmd = "rosrun qbo_mjpeg_server mjpeg_server"
            self.mjpegNode = subprocess.Popen(cmd.split())
            cmd = "rosrun qbo_http_api_login qbo_http_api_login.py"
            self.httpapiNode = subprocess.Popen(cmd.split())
        except:
            return "false"            
        return "true"	

    @cherrypy.expose
    def musicplayer(self):
        try:    
            cmd = "roslaunch qbo_music_player hand_gesture_node.launch"
            self.musicNode = subprocess.Popen(cmd.split())
        except:
            return "false"
        return "true"

    @cherrypy.expose
    def randommove(self):
        try:
            cmd = "roslaunch qbo_random_move random_move_face_demo.launch"
            self.randommoveNode = subprocess.Popen(cmd.split())

        except:
            return "false"
        return "true"

    @cherrypy.expose
    def facetraking(self):
        try:
            cmd = "roslaunch qbo_webi qbo_face_recognition_training.launch"
            self.faceNode = subprocess.Popen(cmd.split())
        except:
            return "false"
        return "true"

    @cherrypy.expose
    def questions(self, lang):
        try:
            cmd = "roslaunch qbo_questions qbo_questions.launch"
            self.questionsNode = subprocess.Popen(cmd.split())
        except:
            return "false"
        return "true"


    @cherrypy.expose
    def calculator(self, lang):
        try:
            cmd = "roslaunch qbo_calculator qbo_calculator_"+lang.upper()+".launch"
            self.calculatorNode = subprocess.Popen(cmd.split())
        except:
            return "false"
        return "true"



    @cherrypy.expose
    def stopFacetraking(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Face Recognizer nodes")
            self.faceNode.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Face Recognizer Process "+str(e))
        return "true"


    @cherrypy.expose
    def stopPhonelauncher(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Phone Launcher nodes")
            self.mjpegNode.send_signal(signal.SIGINT)
            self.httpapiNode.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Phone Launcher process "+str(e))
        return "true"

    @cherrypy.expose
    def stopMusicplayer(self):
        try:
            rospy.loginfo("Qbo Webi: Killin Music Player nodes")
            self.musicNode.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Music Player Process "+str(e))
        return "true"

    @cherrypy.expose
    def stopRandommove(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Random Move nodes")
            self.randommoveNode.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Random Move Process "+str(e))
        return "true"

    @cherrypy.expose
    def stopQuestions(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Qbo Questions node")
            self.questionsNode.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Qbo Questions node "+str(e))
            return "false"
        return "true"


    @cherrypy.expose
    def stopCalculator(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Qbo Calculator node")
            self.calculatorNode.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Qbo Calculator node "+str(e))
            return "false"
        return "true"
