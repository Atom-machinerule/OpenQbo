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
import roslib
import subprocess
from subprocess import Popen, PIPE, STDOUT
import signal
import shlex


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

class ConfWizardManager(TabClass):

    def __init__(self,language):
          self.language = language
          self.htmlTemplate = Template(filename='confWizard/templates/confWizardTemplate.html')


    def localclient(self):
        client=cherrypy.request.remote.ip
        if (client=="127.0.0.1" or client=="localhost" or cherrypy.request.headers['Host'].find(client)!=-1):
                islocal=True
        else:
                islocal=False
        return islocal;

    def getAutoStopValue(self):
       AutoStopVal = rospy.get_param('/qbo_arduqbo/autostop', False)
       return AutoStopVal

    @cherrypy.expose
    def index(self):
        return self.htmlTemplate.render(language=self.language, localclient=self.localclient(), autostopvalue=self.getAutoStopValue())

    @cherrypy.expose
    def camera_calib(self):
        if not self.localclient():
            return "false"

        try:
            self.camera_calibration = subprocess.Popen(shlex.split('gnome-terminal -x bash -c "roslaunch qbo_camera qbo_cameras_stereo_calibration.launch"'))
        except:
            return "false"            
        return "true"	

    @cherrypy.expose
    def hand_gesture_calib(self):
        if not self.localclient():
            return "false"

        try:    
            self.hand_gesture_calibration = subprocess.Popen(shlex.split('gnome-terminal -x bash -c "roslaunch qbo_music_player hand_gesture_calib.launch"'))
        except:
            return "false"
        return "true"


    @cherrypy.expose
    def set_auto_stop_srv(self, autostopval):

        if autostopval == "true":
           rospy.set_param('/qbo_arduqbo/autostop', True)
           rospy.loginfo("Autostop was set true")
        else:
           rospy.set_param('/qbo_arduqbo/autostop', False)
           rospy.loginfo("Autostop was set false")

        return "true"

    @cherrypy.expose
    def create_user(self,userName,newPassword1,newPassword2):
        #if not self.localclient():
            #return "false"

        if newPassword1 != newPassword2:
            return "-2"        

        path = roslib.packages.get_pkg_dir("qbo_http_api_login")

        #we create a temporally dicctionary from users_pwd file
        usersAndPasswords = {}
        f = open(path+'/config/users_pwd')
        for line in f.readlines():
            parts = line.split(" ")
            usersAndPasswords[ parts[0] ] = parts[1].replace("\n","")

        f.close()

        #add password to user
        if userName in usersAndPasswords:
                return "-1"
        else:
            usersAndPasswords[userName] = newPassword1


        f = open(path+'/config/users_pwd','w')
        #from dict to file
        for name in usersAndPasswords:
            f.write(name+" "+usersAndPasswords[name]+"\n")


    @cherrypy.expose
    def save_password(self,userName,oldPassword,newPassword1,newPassword2):
        #if not self.localclient():
            #return "false"

        if newPassword1 != newPassword2:
            return "-2"
        
        path = roslib.packages.get_pkg_dir("qbo_http_api_login")

        #we create a temporally dicctionary from users_pwd file
        usersAndPasswords = {}
        f = open(path+'/config/users_pwd')
        for line in f.readlines():
            parts = line.split(" ")
            usersAndPasswords[ parts[0] ] = parts[1].replace("\n","")

        f.close()

        #change/add password to user
        if userName in usersAndPasswords:
            print userName+" -------------- "+oldPassword+" --------"+  usersAndPasswords[userName]
            if oldPassword == usersAndPasswords[userName]:        
                usersAndPasswords[userName] = newPassword1    
            else:
                return "-1"
        else:
            return "-3"           


        f = open(path+'/config/users_pwd','w')
        #from dict to file
        for name in usersAndPasswords:
            f.write(name+" "+usersAndPasswords[name]+"\n")


    @cherrypy.expose
    def get_list_users(self):
        #if not self.localclient():
            #return "false"

        path = roslib.packages.get_pkg_dir("qbo_http_api_login")

        #we create a temporally dicctionary from users_pwd file
        users = ""
        f = open(path+'/config/users_pwd')
        for line in f.readlines():
            parts = line.split(" ")
            users = parts[0]+":::"+users

        f.close()

        return users


    @cherrypy.expose
    def change_robot_password(self,suPass,newPass1,newPass2):
        path = roslib.packages.get_pkg_dir("qbo_webi")+'/src/robotpass'

        if not self.localclient():
            return "false"

        if newPass1==newPass2:
            if newPass2=='':
                return "false"
            result=runCmd("echo "+suPass+" | sudo -S echo test")
            if result[2]==0:
                result=runCmd("echo "+suPass+" | sudo -S echo "+newPass1+" > "+path)
            print result
            return str(result[2])

        return "false"

