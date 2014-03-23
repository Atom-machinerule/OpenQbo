#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012 OpenQbo, Inc.
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
#

# -*- coding: utf-8 -*-

import os
import signal
import subprocess
import socket
import threading
import time
import re


class ServerError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class FestivalServer(threading.Thread):
    def __init__(self):
        # initialize the process
        threading.Thread.__init__(self)
        self._process = None
        #self._language = "spanish"
        self._festival_bin = "festival"
        try:
            self.start()
        except ServerError, e:
            pass

    def run(self):
        #args = [self._festival_bin, "--server", "--language", self._language]
        args = [self._festival_bin, "--server"]
        self._process = subprocess.Popen( args, 
                              stdin = subprocess.PIPE, 
                              stdout = subprocess.PIPE, 
                              stderr = subprocess.PIPE,
                              close_fds = True) 
        stdout, stderr = self._process.communicate() 
        
        if stderr.rstrip() == "socket: bind failed":
            raise ServerError(stderr.rstrip())
        
        #self._pid = os.spawnlp(os.P_NOWAIT, "festival","festival", "--server")
        #print self._pid
            
    def stop(self):
        """Kill the instance of the festival server.
        
        """
        if self.get_pid() != None:
            try:
                os.kill(self.get_pid(), signal.SIGTERM)
            except OSError:
                print "Error killing the festival server"
        else:
            print "No festival server process to stop"
            
    def restart(self):
        self.stop()
        self.start()
    
    def get_pid(self):
        if self._process != None:
            return self._process.pid
        else:
            return None

class FestivalClient(object):
    def __init__(self, port=1314, host="127.0.0.1"):
        self._count = 0;
        self._talking = 0;
        self.lock = threading.Lock()
        self._host = host
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(0.2)
        self.duration = 1.0
        self.voice = "cmu_us_slt_arctic_clunits"
        
    def open(self):
        self._sock.connect((self._host, self._port))
        
    def close(self):
        self._sock.close()
        
    def send(self, cmd):
        print "Sending "+cmd+" ..."
        self._sock.send(cmd)
        print "Sended "+cmd
    
    def recv(self):
        data = ''
        try:
            data = self._sock.recv(1024)
        except Exception:
            data = ''
        print data, 'len: ',len(data)
        return data
    
    def say(self, text, wait=True):
        print "A new string arrived:"+text
        with self.lock:
            while len(self.recv())>0:
                print 'Full!!'
            args0 = ["echo", text]
            args1 = ["iconv", "-f", "utf-8", "-t", "iso-8859-1"]
            print "1"
            tmpcount=self._count;
            self._count=self._count+1
            print "2"
            p0 = subprocess.Popen( args0, 
                              stdout = subprocess.PIPE,
                              close_fds = True) 
            print "3"
            p1 = subprocess.Popen( args1, 
                              stdin = p0.stdout, 
                              stdout = subprocess.PIPE, 
                              stderr = subprocess.PIPE,
                              close_fds = True) 
            print "4"
            text, stderr = p1.communicate()
            text='(SayText "' + text.strip() + '")'
            print "start speech:"+str(tmpcount)
            print text
            self.send(text)
            if wait:
                print 'wait til talk compleated'
                self.waitEndSpeach()
            else:
                t = threading.Thread(target=self.waitEndSpeach)
                t.start()
            print "end speech: "+str(tmpcount)
        return "OK"

    def waitEndSpeach(self):
        try:
            data=self._sock.recv(1024)
        except Exception:
            data = ''
        print "start--"+data+"--end"
        #if re.search("LP", data) != None:
        #    while re.search("OK", data) == None:
        #        data = self._sock.recv(1024)

        #while re.search("Utterance", data) == None:
        timeNow = time.time()
        timeStart = timeNow
        while re.search("ft_StUfF_keyOK", data) == None and (timeNow-timeStart)<30.0:
            try:
                data = data+self._sock.recv(1024)
            except Exception:
                data = ''
            print "start--"+data+"--end"
            time.sleep(.01)
            timeNow = time.time()
        if (timeNow-timeStart)>=30.0:
            print "Utterance not found"
            return False
        else:
            print "Utterance found"
            return True

    def setDuration(self, duration):
        self.duration = duration
        command = "(Parameter.set 'Duration_Stretch " + str(self.duration) + ")"
        self.send(command)
        self.recv()
        return "OK"

    def setVoice(self, voice):
        self.voice = voice
        command = "(voice_" + self.voice + ")"
        self.send(command)
        self.recv()
        return "OK"
    
def test_server_client():
    festSev = FestivalServer()
    time.sleep(2)
    festCli = FestivalClient()
    festCli.open()
    #time.sleep(0.5)
    #festCli.setVoice("el_diphone")
    festCli.say("Helo")
    #festCli.setDuration(1)
    print "waiting 5"
    time.sleep(5)
    print "talk again"
    festCli.say("Helo Miguel, how are you?")
    time.sleep(5)
    festCli.say("This is a test")
    time.sleep(5)
    festCli.close()
    festSev.stop()
    print "end"
    return

if __name__=="__main__":
    test_server_client()
