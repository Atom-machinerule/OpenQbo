#!/usr/bin/env python2.6
# -*- coding: utf-8 -*-
#!/usr/bin/env python
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
import gen_grammar
import subprocess
from mako.template import Template
from tabsClass import TabClass

import simplejson

from subprocess import Popen, PIPE, STDOUT

import roslib
import signal
roslib.load_manifest('qbo_webi');

import rospy

import time

from uuid import getnode as get_mac
from poster.encode import multipart_encode
from poster.streaminghttp import register_openers
import urllib2

class VoiceRecognitionManager(TabClass):



    def __init__(self,language):
       

        self.ipWavServer = "audio.openqbo.org"
        self.portWavServer="8588"
        self.language = language
        self.juliusPath=roslib.packages.get_pkg_dir("qbo_listen")
        self.juliusAMPath="/usr/share/qbo-julius-model/"
        self.htmlTemplate = Template(filename='voiceRecognition/templates/voiceRecognitionTemplate.html')
        self.jsTemplate = Template(filename='voiceRecognition/templates/voiceRecognitionTemplate.js')
        self.tmpdir="/tmp/"
        self.LMPaths="/config/LM/"
        self.LMFileName="/sentences.conf"
        self.PhonemsFileName="/phonems"
        self.TiedlistFileName="/tiedlist"
        self.languages_names={'en':'English','es':'Spanish','pt':'Português','de':'Deutsch','fr':'Français','it':'Italiano'}
        self.path = roslib.packages.get_pkg_dir("qbo_webi")+"/src/voiceRecognition/"
        self.lan = self.language["current_language"]
        self.mac = get_mac()
        self.p = None


    @cherrypy.expose
    def voiceRecognitionJs(self, parameters=None):
        self.lan = self.language["current_language"]
        return self.jsTemplate.render(language=self.language)


    def getLanguages(self):
        try:
            dirList=os.listdir(self.juliusPath+self.LMPaths)
            dirList.sort()
        except:
            dirList=-1
        return dirList    


    def isQboListenInstalled(self):
        if self.getLanguages()==-1:
            return False
        else:
            return True

    def getLanguageModels(self,language):
        try:
            dirList=os.listdir(self.juliusPath+self.LMPaths+language)
            dirList.sort()
        except:
            dirList=-1
        return dirList

    def getLMSentences(self,language,model):
        try:
            f = open(self.juliusPath+self.LMPaths+language+"/"+model+self.LMFileName,'r')
            return f.read()
        except:
            sentences=""
        return sentences

    @cherrypy.expose
    def getModels(self,lang):
        modelList=""
        try:
            dirList=os.listdir(self.juliusPath+self.LMPaths+lang)
            dirList.sort()
            for model in dirList:
                modelList=modelList+model+"::"
            modelList=modelList[:-2]
        except:
            modelList=-1
        return modelList

    @cherrypy.expose
    def test1(self,lang,text):
        text=text.encode("utf-8")
        f = open(self.tmpdir+'LModel', 'w')
        f.write(text)
        f.close()
        words=gen_grammar.verrors(self.tmpdir+'LModel',self.juliusAMPath+lang+"/"+self.PhonemsFileName)
        if words==0:
             return ""
        else:
            wordsList=""
            for word in words:
                wordsList=wordsList+word+"::"
            wordsList=wordsList[:-2]
            return wordsList

    @cherrypy.expose
    def test2(self,lang,text):
        errorlist=""
        text=text.encode("utf-8")
        print text
        wordlist=text.split()
        print wordlist
        for word in wordlist:
            if word[0]!="[" and word[0]!="<":
                print word
                f = open(self.tmpdir+'word', 'w')
                f.write("[sentence]\n")
                f.write(word)
                f.close()
                gen_grammar.createvoca(self.tmpdir+'word', self.juliusAMPath+lang+"/"+self.PhonemsFileName, self.tmpdir+'word')
                print self.tmpdir+'word'
                print self.juliusAMPath+lang+"/"+self.TiedlistFileName
                if gen_grammar.perrors(self.tmpdir+'word.voca',self.juliusAMPath+lang+"/"+self.TiedlistFileName)!=0:
                    errorlist=errorlist+word+"::"
        errorlist=errorlist[:-2]
        return errorlist.upper()
    
    @cherrypy.expose
    def saveToFile(self,lang,text,model):
        try:
            #print self.juliusPath+self.LMPaths+language+"/"+model+self.LMFileName
            text=text.encode("utf-8")
            f = open(self.juliusPath+self.LMPaths+lang+"/"+model+self.LMFileName,'w')
            f.write(text)
            f.close()
            gen_grammar.compilegrammar(model,lang)
            subprocess.Popen("roslaunch qbo_listen voice_recognizer.launch".split())
        except:
            return "ERROR: Cant write the file"
        return ""

    @cherrypy.expose
    def getFile(self,lang="",model=""):
        if lang=="" or model=="":
            return "ERROR: lang:"+lang+"; model:"+model
        else:
            #print self.getLMSentences(lang,model)
            return self.getLMSentences(lang,model)


    @cherrypy.expose
    def index(self):
        tmp=""
        if self.isQboListenInstalled():
            for lang in self.getLanguages():
                for LM in self.getLanguageModels(lang):
                    text= self.getLMSentences(lang,LM)
                    break
                break

            return self.htmlTemplate.render(language=self.language,lannames=self.languages_names,alllanguage=self.getLanguages())
        else:
            return "Qbo listen not installed"
#        return self.htmlTemplate.render(language=self.language)






    @cherrypy.expose
    def rec(self):

        #   n = self.getLenght("Arturo","sp")
        #   print "***** "+n

        #Borramos la anterior grabacion, si la habia
        try:
            cmd="rm "+self.path+"tmp/*"
            self.p = Popen(cmd.split())
        except ValueError:
            print "Nada que borrar"

        '''
        try:    
            cmd="rm "+self.path+"/*_en"
            self.p = Popen(cmd.split())
        except ValueError:
            print "Nada que borrar"

        try:
            cmd="rm "+path+"/*sp"
            print cmd
            self.p = Popen(cmd.split())

        except ValueError:
            print "Nada que borrar"
        '''


        
        self.filename = str(self.mac)+"_"+self.lan
        #filename = filename.replace("\"","")

    #   filename = "tmp.wav"

        print "FILENAME == "+self.filename

        print "grabnando!!!! "+self.path+"tmp/"+self.filename
        cmd="arecord -f S16_LE  -r 44100 -c 1 "+self.path+"tmp/"+self.filename

        self.p = Popen(cmd.split())

        name="oleole"
        return name


    @cherrypy.expose
    def stop(self):


        if(self.p==None):
            print "P ES NULL!!??"
        else:
            print "matar grabacin"
            self.p.send_signal(signal.SIGINT)

        cmd="python "+self.path+"sendWav2Server.py "+self.path+"tmp/"+self.filename+" "+self.ipWavServer+" "+self.portWavServer
        print cmd
        out = runCmd(cmd)

        print out[0]

        if out[1] != "":
            print "Error"
            return "error"


        return unicode(out[0],'utf8')

     



    @cherrypy.expose
    def play(self):
        print "play sound"  

        os.system('aplay '+self.path+"tmp/"+self.filename)

        return "ok"


    @cherrypy.expose
    def save(self,transcripcion):
        print "SAVE! transcripcion="+transcripcion


        cmd="python "+self.path+"sendTranscription2Server.py "+str(self.mac)+" \""+transcripcion+"\" "+self.lan+" "+self.ipWavServer+" "+self.portWavServer
        print cmd
        out = runCmd(cmd)
        

        if out[1] != "":
            print "Error "+out[1]
            return "error"

        return out[0]

#        return "ok"
    




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




