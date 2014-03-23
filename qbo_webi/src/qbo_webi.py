#!/usr/bin/env python
# coding: utf-8
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

#ROS stuff
import roslib
roslib.load_manifest('qbo_webi');

import rospy

#Other imports
import cherrypy
from mako.template import Template
import json
from sysChecks.sysChecks import sysChecksManager
from training.FaceObjectTrainer import FaceObjectTrainer
from settings.settings import SettingsManager
#CHANGED
from test.test import TestManager
from teleoperation.teleoperation import TeleoperationManager
from confWizard.confWizard import ConfWizardManager
from launchersTab.launchersTab import LaunchersTabManager
from otherFunctionalities.mjpegServerFuntions.MjpegServerFunctions  import MjpegServerFunctions
from voiceRecognition.voiceRecognition import VoiceRecognitionManager
from qbo_questions.qbo_questions import Qbo_questionsManager
from recorder.recorder import RecorderManager
from xmms2.xmms2 import XMMS2Manager
from mjpeg.mjpeg import MjpegGrabber
import os
import sys
import signal

from std_msgs.msg import String

from auth import AuthController, require, member_of, name_is

pathh = '/'.join(os.path.abspath( __file__ ).split('/')[:-1])
os.chdir(pathh)

rospy.loginfo("Webi: "+pathh)

class Root(object):

    _cp_config = {
        'tools.sessions.on': True,
        'tools.auth.on': True,
        'tools.sessions.locking': 'explicit'
    }
    
    auth = AuthController()

    def __init__(self):
        self.indexHtmlTemplate = Template(filename='templates/indexTemplate.html')

        self.lang = rospy.get_param("/system_lang", "en")
    
        #self.lang='en'
        #Load default dict
        fp=open('lang/en.txt','r')
        self.language=json.load(fp,'utf-8')
        fp.close()
        #Load specific dict if different to english
        if self.lang!='en':
            try:
                fp=open('lang/'+self.lang+'.txt','r')
#            self.language.update(json.load(fp))

                self.language.update(json.load(fp))
                fp.close()
            except IOError:
                print "Language error"
                self.lang = "en"

        #Post the system lang to ROS and set the Param
        if self.lang == "en" or self.lang == "es" :
            rospy.set_param("/system_lang", self.lang)
            lang_pub = rospy.Publisher('/system_lang', String)
            lang_pub.publish(String(self.lang))
            print "PUBLISHING NEW LANG: "+self.lang

        '''
        else: #If language is unknown, set the ROS system language as english
            rospy.set_param("/system_lang", "en")
            lang_pub = rospy.Publisher('/system_lang', String)
            lang_pub.publish(String("en"))
        '''

    def readLanFile(self):
        config=ConfigParser.ConfigParser()
        config.readfp(open("lang/en.txt"))
        allsections=config.sections()
        for i in allsections:
            if req.command==config.get(i,"input"):
                execparams=config.get(i, "command")
                result=commands.getoutput(execparams)
                return result
        return "Error, input not exist"


    def setCookieLang(self,lan):
        cookie = cherrypy.response.cookie
        cookie['language'] = lan
        cookie['language']['path'] = '/'
        cookie['language']['max-age'] = 360000
        cookie['language']['version'] = 1

    def readCookieLang(self):
        cookie = cherrypy.request.cookie
	try: 
            res=cookie['language'].value
        except KeyError:
            print "KEY ERROR"
            res=""
        return res

    @cherrypy.expose
    def index(self,new_lang="",activeTab=0):
        print "NEW LANG CHANGE REQUEST: "+new_lang

        if new_lang!="":
            self.change_language(new_lang)
        '''
        else:
            cookielang=self.readCookieLang()
            if cookielang!="":
                self.change_language(cookielang)
        '''
        return self.indexHtmlTemplate.render(language=self.language,tab=activeTab)

    #@cherrypy.expose
    #def test(self):
        #return "Login needed test service"

    def change_language(self, new_lang):
        #Load specific dict
        #Load default dict
        print "Comparing self.lang: "+self.lang+", new_lang: "+new_lang
        if self.lang!=new_lang:
            #self.setCookieLang(new_lang)
            fp=open('lang/en.txt','r')
            self.language=json.load(fp,'utf-8')
            fp.close()
            try:
                fp=open('lang/'+new_lang+'.txt','r')
#            self.language.update(json.load(fp))
                self.language.update(json.load(fp))
                fp.close()
                self.lang = new_lang
            except IOError:
                print "Language error"

            #Post the system lang to ROS and set the Param
            if new_lang == "en" or new_lang == "es":
                rospy.set_param("/system_lang", new_lang)
                lang_pub = rospy.Publisher('/system_lang', String)                
                lang_pub.publish(String(new_lang))
                print "PUBLISHING NEW LANG: "+new_lang
            else: #If language is unknown, set the ROS system language as english
                rospy.set_param("/system_lang", "en")
                lang_pub = rospy.Publisher('/system_lang', String)                
                lang_pub.publish(String("en"))
 
        cherrypy.root.checkers.set_language(self.language)
        cherrypy.root.training.set_language(self.language)
        cherrypy.root.settings.set_language(self.language)
        cherrypy.root.teleoperation.set_language(self.language)
        cherrypy.root.confWizard.set_language(self.language)
        cherrypy.root.voiceRecognition.set_language(self.language)
        cherrypy.root.xmms2.set_language(self.language)
        cherrypy.root.qbo_questions.set_language(self.language)
        cherrypy.root.launchersTab.set_language(self.language)
        cherrypy.root.recorder.set_language(self.language)
        #Reload the checkers dictionary
#        return "HOLA"+new_lang


def SIGINT_handler(signal, frame):
    print "SigInt Recived"
    rospy.signal_shutdown("ROS Node kill was sent by exterior process")
    sys.exit(0)

#Initialize ROS node associated with Q.bo Webi
rospy.init_node(name="qbo_webi", argv=sys.argv, disable_signals=True)
signal.signal(signal.SIGINT, SIGINT_handler)


cherrypy.root = Root()
cherrypy.root.checkers = sysChecksManager(cherrypy.root.language)
cherrypy.root.training = FaceObjectTrainer(cherrypy.root.language)
cherrypy.root.settings = SettingsManager(cherrypy.root.language)
cherrypy.root.teleoperation = TeleoperationManager(cherrypy.root.language)
cherrypy.root.confWizard = ConfWizardManager(cherrypy.root.language)
cherrypy.root.voiceRecognition = VoiceRecognitionManager(cherrypy.root.language)
cherrypy.root.xmms2 = XMMS2Manager(cherrypy.root.language)
cherrypy.root.launchersTab = LaunchersTabManager(cherrypy.root.language)
cherrypy.root.mjpegServer = MjpegServerFunctions() 
cherrypy.root.image = MjpegGrabber()
cherrypy.root.qbo_questions = Qbo_questionsManager(cherrypy.root.language)
cherrypy.root.recorder = RecorderManager(cherrypy.root.language)
cherrypy.root.test = TestManager(cherrypy.root.language)




#Get ROS parameter of the server Port
server_port = rospy.get_param("server_port", 7070)

conf = {
    'global': {
        'server.socket_host': '0.0.0.0',
        'server.socket_port': server_port,
        #'server.ssl_module':'pyopenssl',
        #'server.ssl_certificate':'/home/qboblue/keys/server.crt',
        #'server.ssl_private_key':'/home/qboblue/keys/server.key.insecure',
        #'server.ssl_certificate_chain':'/home/qboblue/keys/server.crt'
    },

    '/favicon.ico': {'tools.staticfile.on': True,
        'tools.staticfile.filename': pathh+'/img/favicon.ico'},

    '/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/static/img'},

        '/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/static/js'},

        '/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/static/css'},

        '/training/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/training/static/img'},

        '/training/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/training/static/js'},

        '/training/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/training/static/css'},

        '/settings/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/settings/static/img'},

        '/settings/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/settings/static/js'},

        '/settings/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/settings/static/css'},

        '/confWizard/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/confWizard/static/img'},

        '/confWizard/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/confWizard/static/js'},

        '/confWizard/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/confWizard/static/css'},

        '/teleoperation/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/teleoperation/static/img'},

        '/teleoperation/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/teleoperation/static/js'},

        '/teleoperation/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/teleoperation/static/css'},

        '/teleoperation/sip': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/teleoperation/sip2rtmp'},

        '/launchersTab/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/launchersTab/static/img'},

        '/launchersTab/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/launchersTab/static/js'},

        '/launchersTab/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/launchersTab/static/css'},

        '/qbo_questions/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/qbo_questions/static/img'},

        '/qbo_questions/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/qbo_questions/static/js'},

        '/qbo_questions/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/qbo_questions/static/css'},

        '/sysChecks/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/sysChecks/static/img'},

        '/sysChecks/static/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/sysChecks/static/js'},

        '/sysChecks/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/sysChecks/static/css'},

        '/xmms2/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/xmms2/static/img'},
        
        '/xmms2/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/xmms2/static/css'},

        '/voiceRecognition/static/css': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/voiceRecognition/static/css'},

        '/voiceRecognition/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/voiceRecognition/static/img'},


        '/recorder/static/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/recorder/static/img'},

        '/recorder/videos': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/recorder/videos'},

        '/test/js': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/test/static/js'},

        '/test/img': {'tools.staticdir.on': True,
        'tools.staticdir.dir': pathh+'/test/static/img'},


}


def roskill_handler():
    print "qbo_webi ROS node is shuting down"
    rospy.signal_shutdown("ROS Node kill was sent by exterior process")
    cherrypy.engine.exit()
    #sys.exit(0)


rospy.on_shutdown(roskill_handler)
cherrypy.quickstart(cherrypy.root, '/', conf)

