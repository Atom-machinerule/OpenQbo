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

import rospy
import cherrypy
import os
import subprocess
from subprocess import Popen, PIPE, STDOUT
import signal

def runCmd(cmd, timeout=None):
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
                         stderr=subprocess.PIPE)
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

class MjpegServerFunctions(object):


    def __init__(self):

        self.threeDOn = False

        self.urls = {'leftEye':'/image/snapshot?topic=/stereo/left/image_raw&quality=',
	            'rightEye':'/image/snapshot?topic=/stereo/right/image_raw&quality=',
	            #'rightEye':'/image/snapshot?topic=/camera/rgb/image_mono&quality=',
	            'objects':'/image/snapshot?topic=/qbo_stereo_selector/viewer&quality=',
    	        'faces':'/image/snapshot?topic=/qbo_face_tracking/viewer&quality=',
    	        'live_leftEye':'/image/stream?topic=/stereo/left/image_raw&quality=',
                'live_rightEye':'/image/stream?topic=/stereo/right/image_raw&quality=',
                'live_3d':'/image/stream?topic=/stereo_anaglyph&quality=',
                'live_objects':'/image/stream?topic=/qbo_stereo_selector/viewer&quality=',
                'live_faces':'/image/stream?topic=/qbo_face_tracking/viewer&quality=',

        }


    '''
    @cherrypy.expose
    def start(self,port=8081):            
               return 
               rospy.loginfo("Launching MJPEG Server")
               
               if self.isServerRunning():
                   rospy.loginfo("MJPEG Server running")
                   return
               #cmd = "rosrun qbo_mjpeg_server mjpeg_server &"
               cmd = "roslaunch qbo_mjpeg_server mjpeg_server"+str(port)+".launch"
               
               if(self.port==8081):
                   self.processMjpegServer8081 = subprocess.Popen(cmd.split())
               elif(self.port==8082):
                   self.processMjpegServer8082 = subprocess.Popen(cmd.split())
    '''

    '''
    @cherrypy.expose
    def stop(self,port):
        rospy.loginfo("MJPEG Server does not stop")
        return 'ok'
        rospy.loginfo("Stopping MJPEG Server")
        try:
            if(port=="8081"):
                self.processMjpegServer8081.send_signal(signal.SIGINT)            
            elif(port=="8082"):
                self.processMjpegServer8082.send_signal(signal.SIGINT)
        

            if self.threeDOn:                
                self.stopAnaglyph()

            return "OK"
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill process MJPEG Server. "+str(e))
            return "ERROR"
    '''
    '''
    def isServerRunning(self):
        ou=runCmd('rosnode list | grep mjpeg_server')[0].strip().split()
        if len(ou)>0:
            return True
        else:
            return False
    '''


    # input params:
    #  - image: String with the key to get the type of image
    #  - quality: Int from 0 to 100. 100 is the best quality
    #  - widht: Int with the image width desired
    #  - height: Int with the image height desired
    #
    # output:
    # - -1 if something went wrong, like asking for an kind of image not supported
    # - The string with the URL to that topic
    @cherrypy.expose
    def getUrlFrom(self,image,quality,width,height):
        print "DAME "+image
        if("3d" in image):
            self.threeDOn = True
            cmd="rosrun stereo_anaglyph red_cyan_anaglyph.py __name:=stereo_anaglyph -c /stereo -d 20 -s"
            self.processthreeD = subprocess.Popen(cmd.split())
            
            url = self.urls[image]
            finalUrl = url+quality+'&width='+width+'&height='+height,

            return finalUrl

        else:
            if(self.threeDOn):
                print "------  Antes estaba 3d, ahora cambiamos de imagen "
                self.stopAnaglyph()

            print "1"

    	    try:
                url = self.urls[image]
                print "2"
            except Exception as e:
                print "Error al pedir imagen "+image
                return -1

            finalUrl = url+quality+'&width='+width+'&height='+height
            
            return finalUrl

    '''
    def getStreamSrc(self,image,quality,width,height):
        sourc=self.getUrlFrom(image,quality,width,height)[0]
        sourc='http://'+self.ip+':'+str(self.port)+sourc.replace('snapshot','stream')
        print 'source: ',sourc
        return sourc
    '''

		
    def stopAnaglyph(self):
        self.processthreeD.send_signal(signal.SIGINT)
        self.threeDOn = False

