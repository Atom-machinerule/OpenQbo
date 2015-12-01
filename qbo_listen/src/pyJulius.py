#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012-2013 TheCorpora SL
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
# Authors: Sergio Merino <s.merino@openqbo.com>;

import os
import signal
import subprocess
import socket
import threading
import time

class JuliusServer(threading.Thread):
    def __init__(self, configFile):
        # initialize the process
        threading.Thread.__init__(self)
        self._process = None
        self._julius_bin = "/usr/local/bin/julius-4.2"
        self._config_file=configFile
        try:
            self.start()
        except ServerError, e:
            pass

    def run(self):
        #args = [self._julius_bin, "-module", "-input", "mic", "-C", self._config_file]
        args = [self._julius_bin, "-input", "mic", "-C", self._config_file]
        self._process = subprocess.Popen( args, 
                              #stdin = subprocess.PIPE, 
                              stdout = subprocess.PIPE, 
                              stderr = subprocess.PIPE) #,
                              #close_fds = True) 
        #stdout, stderr = self._process.communicate() 
        #stdout = self._process.communicate()
        #print 'Tipo: ', type(stdout) 
        #if stderr.rstrip() == "socket: bind failed":
        #    raise ServerError(stderr.rstrip())
        #while True:
        #    readed=self._process.stdout.readline()
        #    print readed
            
    def stop(self):
        """Kill the instance of the julius server.
        
        """
        if self.get_pid() != None:
            try:
                os.kill(self.get_pid(), signal.SIGTERM)
            except OSError:
                print "Error killing the julius server"
        else:
            print "No julius server process to stop"
            
    def restart(self):
        self.stop()
        self.start()
    
    def get_pid(self):
        if self._process != None:
            return self._process.pid
        else:
            return None
