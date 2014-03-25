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
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
import os
from os import environ
import subprocess
from subprocess import Popen, PIPE, STDOUT
import signal

class RunCmd(object):


    def __init__(self):
	self.env = environ.copy()
        pass

    def addToPythonPath(self, path):	
	self.env["PYTHONPATH"]=path+self.env["PYTHONPATH"]

    def getPythonPath(self):
	return self.env["PYTHONPATH"]

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



    def runCmdWithPidBack(self, cmd):
	return subprocess.Popen(cmd.split(),env=self.env)

    def killProcess(self, process):
        try:
            process.send_signal(signal.SIGINT)
        except Exception as e:
            print "ERROR when killing a proccess. "+str(e)

