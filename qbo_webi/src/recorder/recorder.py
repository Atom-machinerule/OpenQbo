#!/usr/bin/env python2.6
# coding: utf-8
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
import glob
from mako.template import Template
from tabsClass import TabClass
import rospy
import subprocess
import os
import signal
import roslib; roslib.load_manifest('qbo_webi')
from qbo_video_record.srv import *

class RecorderManager(TabClass):
    
    def __init__(self,language):
        self.language = language
        self.htmlTemplate = Template(filename='recorder/templates/recorderTemplate.html')
        self.jsTemplate = Template(filename='recorder/templates/recorderTemplate.js')
        self.processname = 'qbo_video_record'
        self.videosDir=roslib.packages.get_pkg_dir("qbo_webi")+"/src/recorder/videos/";

    def get_videos(self):
        videoFiles=[]
        for fname in glob.glob(self.videosDir+"*.ogv"):
            splitdir=fname.split("/")
            videoFiles.append(splitdir[10])
        videoFiles.sort()
        return videoFiles

    @cherrypy.expose
    def video_list_html(self):
        videos=self.get_videos()
        index=0
        html='<ol id="selectable" class="ui-selectable">'
        test=""
        for video in videos:
            #html=html+'<li id="element'+str(index)+'" class="ui-widget-content ui-selectee" name="/recorder/videos/'+video+'" >     <table style="width:100%;"><tr>  <td style="width: 15px;" > </td> <td style="width: 215px;"> <p style="width:215px; margin:0; word-wrap:break-word; overflow: hidden;height: 20px;line-height: 20px;  ">'+video+' </p>   </td> <td onClick="deleteVideo(\''+video+'\');">  <div style="float:right;width:16px;height:16px;" class="BasuraIcon" ></div>   </td>    </tr>          </table>     </li>'


            html=html+'<li class="ui-state-default" onclick="playVideo(\'/recorder/videos/'+video+'\')" id="'+video+'">  <table style="width:100%;"><tr>  <td style="width: 15px;" > </td> <td style="width: 215px;"> <p style="width:215px; margin:0; word-wrap:break-word; overflow: hidden;height: 20px;line-height: 20px;  ">'+video+' </p>   </td> <td> <div onclick="deleteVideo(\''+video+'\')" style="float:right;" class="ui-icon ui-icon-trash delete-buttons" ></div>   </td>    </tr>          </table>             </li> '

            index=index+1

        html=html+"</ol>"+test 
        return html

    @cherrypy.expose
    def status(self):
        rospy.wait_for_service('/qbo_video_record/status')
        try:
            status = rospy.ServiceProxy('/qbo_video_record/status', StatusRecord)
            resStatus = status()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return str(resStatus.status)

    @cherrypy.expose
    def record(self):
        status=self.status()
        if (status=="0"):
            rospy.wait_for_service('/qbo_video_record/start')
            start=rospy.ServiceProxy('/qbo_video_record/start', StartRecord)
            start("/stereo/left/image_raw",self.videosDir)
            return "1"
        else:
            rospy.wait_for_service('/qbo_video_record/stop')
            stop=rospy.ServiceProxy('/qbo_video_record/stop', StopRecord)
            stop()
            return "0"


    @cherrypy.expose
    def deleteVideo(self, videoName):
        print "-------------------"
        os.remove(self.videosDir+videoName)
        print "--------------2"
        return self.video_list_html()


    @cherrypy.expose
    def recorderJs(self, **params):
        return self.jsTemplate.render(language=self.language)

    @cherrypy.expose
    def index(self):
        return self.htmlTemplate.render(language=self.language)
