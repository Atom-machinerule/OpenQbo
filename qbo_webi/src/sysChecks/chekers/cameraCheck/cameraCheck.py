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
# Authors: Miguel Angel Julian <miguel.julian@openqbo.com>;
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
#          Arturo Bajuelos <arturo@openqbo.com>; 
#          Sergio Merino <s.merino@openqbo.com>;

import cherrypy
from mako.template import Template
from mako.lookup import TemplateLookup
from sysChecks.chekers.chekers import chekers
import subprocess
import time

#from  otherFunctionalities.mjpegServerFuntions.MjpegServerFunctions  import MjpegServerFunctions

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

class cameraCheck(chekers):

    def __init__(self,lang):
        chekers.__init__(self,'cameraCheck',lang)
        self.templatelookup = TemplateLookup(directories=['./'])
        self.htmlTmpl = Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraCheckTemplate.html', lookup=self.templatelookup)

        #check if 2 cameras pressent
        self.camerasCount=self.getDetectedCameras()
        #check if cameras node loaded
        self.camerasNode=self.nodeExist('uvc_camera_stereo')
        #check if stereo node loaded
        self.stereoNode=self.nodeExist('stereo_proc')
        #print '#cams:',self.camerasCount,'camsNode:',self.camerasNode,'stereoNode:',self.stereoNode
        #self.result='fail'
        self.leftWorks=None
        self.leftNotWrong=None
        self.rightWorks=None
        self.rightNotWrong=None

        self.failString=''

        if self.camerasCount < 2 or not self.camerasNode or not self.stereoNode:
            self.result='fail'

        #self.mjpegServer = MjpegServerFunctions()


    def get_html_content(self,failString='camCount'):
        if self.result != 'fail':
            #self.mjpegServer.start()
            #imgSrc=self.mjpegServer.getStreamSrc('leftEye','60','320','240')
            #imgSrc='http://192.168.4.104:'+str(self.mjpegServer.port)+self.mjpegServer.getUrlFrom('leftEye','60','320','240')
            #return self.htmlTmpl.render(language=self.language, imgSrc=imgSrc, stepNumber=0)
            stepNumber=-1
            if self.rightNotWrong:
                stepNumber=3
            elif self.rightWorks:
                stepNumber=2
            elif self.leftNotWrong:
                stepNumber=1
            elif self.leftWorks:
                stepNumber=0
            return self.step([stepNumber,'true'])
        else:
            if self.camerasCount==0:
                #No cameras detected. Check if cammeras are connected
                self.failString='camCount'
            elif self.camerasCount==1:
                #Only one camera detected. Check if the two cameras are connected
                self.failString='camCount'
            elif not self.camerasNode:
                #Camera node is not running. Check if ....
                self.failString='camNode'
            elif not self.stereoNode:
                #Stereo node is not running. Check if calibration is done
                self.failString='stereoNode'
            return Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraFailTemplate.html', lookup=self.templatelookup).render(language=self.language, failString=self.failString, camerasCount=self.camerasCount, resultImgSrc='/img/wrong.png', stepNumber=0, imgSrc='/image/noimage')

    def restart(self,params):
        self.leftWorks=None
        self.leftNotWrong=None
        self.rightWorks=None
        self.rightNotWrong=None

        self.result='waiting'

        #check if 2 cameras pressent
        self.camerasCount=self.getDetectedCameras()
        #check if cameras node loaded
        self.camerasNode=self.nodeExist('uvc_camera_stereo')
        #check if stereo node loaded
        self.stereoNode=self.nodeExist('stereo_proc')
        #print '#cams:',self.camerasCount,'camsNode:',self.camerasNode,'stereoNode:',self.stereoNode

        self.failString=''

        if self.camerasCount < 2 or not self.camerasNode or not self.stereoNode:
            self.result='fail'

        return self.get_html_content()

    def step(self,params):
        #try:
            resultImgSrc='/img/loading.gif'
            cameraCheckInstructionsSrc='/img/blank.png'
            stepNumber=int(params[0])
            paramResult=True if params[1]=='true' else False
            print 'parameters: ',params
            checkText='works'
            if stepNumber==-1:
                #imgSrc=self.mjpegServer.getStreamSrc('leftEye','60','320','240')
                imgSrc='/image/stream?topic=/stereo/left/image_raw&quality=60&t='+'%.2f' % (time.time())
                checkText='works'
            elif stepNumber==0:
                #imgSrc=self.mjpegServer.getStreamSrc('leftEye','60','320','240')
                imgSrc='/image/stream?topic=/stereo/left/image_raw&quality=60&t='+'%.2f' % (time.time())
                cameraCheckInstructionsSrc='/img/head_leftEyeBox.png'
                checkText='wrong'
                self.leftWorks=paramResult
                if not self.leftWorks:
                    self.result='fail'
                    resultImgSrc='/img/wrong.png'
                    self.failString='leftNotWorks'
                    return Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraFailTemplate.html', lookup=self.templatelookup).render(language=self.language, failString=self.failString, camerasCount=self.camerasCount, resultImgSrc=resultImgSrc, stepNumber=stepNumber, imgSrc='/image/noimage')
            elif stepNumber==1:
                #imgSrc=self.mjpegServer.getStreamSrc('rightEye','60','320','240')
                imgSrc='/image/stream?topic=/stereo/right/image_raw&quality=60&t='+'%.2f' % (time.time())
                checkText='works'
                self.leftNotWrong=paramResult
                if not self.leftNotWrong:
                    self.result='fail'
                    resultImgSrc='/img/wrong.png'
                    self.failString='leftWrong'
                    return Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraFailTemplate.html', lookup=self.templatelookup).render(language=self.language, failString=self.failString, camerasCount=self.camerasCount, resultImgSrc=resultImgSrc, stepNumber=stepNumber, imgSrc='/image/noimage')
            elif stepNumber==2:
                #imgSrc=self.mjpegServer.getStreamSrc('rightEye','60','320','240')
                imgSrc='/image/stream?topic=/stereo/right/image_raw&quality=60&t='+'%.2f' % (time.time())
                cameraCheckInstructionsSrc='/img/head_rightEyeBox.png'
                checkText='wrong'
                self.rightWorks=paramResult
                if not self.rightWorks:
                    self.result='fail'
                    resultImgSrc='/img/wrong.png'
                    self.failString='rightNotWorks'
                    return Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraFailTemplate.html', lookup=self.templatelookup).render(language=self.language, failString=self.failString, camerasCount=self.camerasCount, resultImgSrc=resultImgSrc, stepNumber=stepNumber, imgSrc='/image/noimage')
            else:
                checkText='works'
                self.rightNotWrong=paramResult
                if not self.rightNotWrong:
                    self.result='fail'
                    resultImgSrc='/img/wrong.png'
                    self.failString='rightWrong'
                    return Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraFailTemplate.html', lookup=self.templatelookup).render(language=self.language, failString=self.failString, camerasCount=self.camerasCount, resultImgSrc=resultImgSrc, stepNumber=stepNumber, imgSrc='/image/noimage')
                if self.leftWorks and self.leftNotWrong and self.rightWorks and self.rightNotWrong:
                    imgSrc='/img/ok.png'
                    resultImgSrc='/img/ok.png'
                    return Template(filename='sysChecks/chekers/'+self.name+'/templates/cameraOkTemplate.html', lookup=self.templatelookup).render(language=self.language, stepNumber=stepNumber+1, imgSrc='/image/noimage')
                else:
                    resultImgSrc='/img/wrong.png'
                    imgSrc='/img/wrong.png'
            return self.htmlTmpl.render(language=self.language, imgSrc=imgSrc, stepNumber=stepNumber+1, resultImgSrc=resultImgSrc, checkText=checkText, imgStopSrc=imgSrc.replace('stream','stop'), cameraCheckInstructionsSrc=cameraCheckInstructionsSrc)
        #except Exception, e:
            #print e
            #return 'ERROR'

    def nodeExist(self,node):
        ou=runCmd('rosnode list | grep '+node)[0].strip().split()
        if len(ou)>0:
            return True
        else:
            return False

    def getDetectedCameras(self):
        ou=runCmd('ls /dev/video*')[0].strip().split()
        return len(ou)



