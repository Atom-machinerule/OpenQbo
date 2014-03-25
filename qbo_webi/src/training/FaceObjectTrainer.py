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
# Authors: Arturo Bajuelos <arturo@openqbo.com>; 
#          Sergio Merino <s.merino@openqbo.com>;
#          etc.
import cherrypy
from mako.template import Template
from tabsClass import TabClass
import rospy

import subprocess
from subprocess import Popen, PIPE, STDOUT
import signal

from std_msgs.msg import String
from sensor_msgs.msg  import Image
from qbo_talk.srv import Text2Speach
from qbo_object_recognition.srv import RecognizeObject
from qbo_object_recognition.srv import Teach
from qbo_object_recognition.srv import LearnNewObject
from qbo_object_recognition.srv import Update
from qbo_face_msgs.srv  import GetName
from qbo_face_msgs.srv  import LearnFaces
from qbo_face_msgs.srv  import Train

import roslib.packages


import unicodedata


from  otherFunctionalities.mjpegServerFuntions.MjpegServerFunctions  import MjpegServerFunctions

import time
import urllib
import urllib2_file
import urllib2
import sys, json
import os
import tarfile
import shutil
class FaceObjectTrainer(TabClass):


    def __init__(self,language):
        self.language = language
        #self.mjpegServer = MjpegServerFunctions()
        self.htmlTemplate = Template(filename='training/templates/trainingTemplate.html')
        self.jsTemplate = Template(filename='training/templates/trainingTemplate.js')
        #self.variablesTemplate = Template(filename='static/js/generalVariables.js')
        self.exposed = True
    	self.faceON = False
        self.cloudAddress = 'http://qbocloud.openqbo.org:8800'


    @cherrypy.expose
    def unload(self):
        #self.mjpegServer.stop("8081")
        self.stopNode()
        return "ok"

    @cherrypy.expose
    def index(self):
        #self.mjpegServer.start("8081")
        return self.htmlTemplate.render(language=self.language)

    @cherrypy.expose
    def trainingJs(self, parameters=None):
        return self.jsTemplate.render(language=self.language)

    #@cherrypy.expose
    #def trainingVariables(self, parameters=None):
    #    return self.variablesTemplate.render(language=self.language)

    @cherrypy.expose
    def selectFaceRecognition(self):
	self.faceON = True


    @cherrypy.expose
    def selectObjectRecognition(self):
        self.faceON = False


    @cherrypy.expose
    def launchNodes(self, faceOn):
        
        if faceOn == "true":
            self.faceON = True
            rospy.loginfo("Face Recognition Mode is activated")
        else:
            self.faceON = False
            rospy.loginfo("Object Recognition Mode is activated")
        
        if self.faceON:
            #call face traningi bellow
            rospy.loginfo("Launching Face Recognition Node")
            self.launchFaceNode()
        else:
            rospy.loginfo("Launching Object Recognition Node")
            self.launchObjectNode()


    @cherrypy.expose
    def stopNode(self):
        if self.faceON:
            #call face training bellow
            self.stopFaceNode()
        else:
            self.stopObjectNode()
        return "ok"

    @cherrypy.expose
    def startLearning(self,objectName):
        if self.faceON:
            rospy.loginfo("Called face learning service with name:"+objectName)
            return self.learnFace(objectName)
        else:
            rospy.loginfo("VAMOS A APRENDER UN OBJETO")
            rospy.loginfo("en concreto un "+objectName)
            return self.learnObject(objectName)

    @cherrypy.expose
    def startTraining(self):
	    if self.faceON:
		    #call face traningi bellow
		    rospy.loginfo("Starting Face Training")
		    return self.trainingFaces()
	    else:
		    return self.trainingObjects()


    @cherrypy.expose
    def startRecognition(self):
        if self.faceON:
            #call face recog bellow
	        rospy.loginfo("Starting Face Recognition")
	        return self.startRecognizeFace()
        else:
            rospy.loginfo("Starting Object Recognition")
            returned= self.startRecognizeObject()
            return returned

    @cherrypy.expose
    def sendToCloud(self):
        if self.faceON:
            return "Face Mode is activated." 
        self.send_images_to_cloud()
        return "True"

    @cherrypy.expose
    def startCloudRecognition(self):
        #Get path to the local cloud data base
        object_path=roslib.packages.get_pkg_dir('qbo_webi')+"/config/"+"cloud_objects_"+self.language["current_language"]

        #Check if the objects folder exists and if not, create it
        if not os.path.exists(object_path):
            os.makedirs(object_path)
        
        rospy.loginfo("Qbo Webi: Path for the cloud's objects: "+object_path) 
        
        #If version file does not exists
        if not os.path.isfile(object_path+"/version"):
            #Download latest version
            rospy.loginfo("Qbo Webi: Downloading latest version from the Q.bo Cloud")
            resp = self.download_latest_version(object_path)
            if resp != "ok":
                return json.dumps({'status':'Unable to download latest version from Q.bo Cloud','object':'none'})
        
        else: 
            #Read version file
            version_str = "" 
            try:
                version_file = open(object_path+"/version") 
                version_str = version_file.read().strip()
                version_file.close()
            except IOError as e:
                return json.dumps({'status':'Could not access local files','object':'none'})
            
            #Get latest version of Q.bo Cloud
            rospy.loginfo("Qbo Webi: Getting version number from Q.bo Cloud...")
            resp = self.get_version() 
            if resp=="":
                return json.dumps({'status':'Could not get version number from Q.bo Cloud','object':'none'})
            
            rospy.loginfo("Qbo Webi: Latest version is "+resp+" and local version is "+version_str)

            if resp == version_str: #If you have latest version, don't do anything
                rospy.loginfo("Qbo Webi: Local version is the latest version: "+version_str)
            else:
                rospy.loginfo("Qbo Webi: Downloading latest version from Q.bo Cloud")
                #download latest version
                resp = self.download_latest_version(object_path)
                if resp != "ok":
                    return json.dumps({'status':'Unable to download latest version from Q.bo Cloud'    ,'object':'none'})
                rospy.loginfo("Qbo Webi: Latest version has been succesfully downloaded from Q.bo Cloud")
        
        #Store the previous update_path
        update_path_back = roslib.packages.get_pkg_dir('qbo_object_recognition')+"/objects/objects_db/"
        rospy.loginfo("Qbo Webi: Storing the previous update path: "+update_path_back)
        
        #Change the update_path of the object recognizer
        rospy.set_param("/qbo_object_recognition/update_path", object_path)


        rospy.loginfo("Qbo Webi: Loading object recognition with the cloud object path")
        #Update the objects folder of the object recognizer to the latest version of Q.bo cloud

        rospy.loginfo("Qbo Webi: Waiting for service update...")
        rospy.wait_for_service("/qbo_object_recognition/update")
        rospy.loginfo("Qbo Webi: Update service is ready!")
        update_srv = rospy.ServiceProxy('/qbo_object_recognition/update',Update)
        resultUpdating = update_srv()

        #Call the recognize service of the object recognizer
        
        rospy.loginfo("Qbo Webi: Started object recognizer mode. Waiting for service...")
        rospy.wait_for_service("/qbo_object_recognition/recognize_with_stabilizer")
        rospy.loginfo("Qbo Webi: Recognize service is ready!")
        recog = rospy.ServiceProxy('/qbo_object_recognition/recognize_with_stabilizer',RecognizeObject)
        object_recog = recog()
        

        #Restore the previous update_path and load the previous object daba base
        rospy.set_param("/qbo_object_recognition/update_path", update_path_back)

        rospy.loginfo("Restoring the object recognition node with the previous data base")
        update_srv = rospy.ServiceProxy('/qbo_object_recognition/update',Update)
        resultUpdating = update_srv()

        output = json.dumps({'status':'ok','object':str(object_recog.object_name)})
        
        return output

############ Face Training and Recognition ######################################

    def launchFaceNode(self):
    	rospy.loginfo("Qbo Webi: Launching Face Recognizer nodes")
        cmd = "roslaunch qbo_webi qbo_face_recognition_training.launch"
        self.processFaceNode = subprocess.Popen(cmd.split())
        rospy.loginfo(" END Launching Face Recognizer nodes")


    def stopFaceNode(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Face Recognizer nodes")
            #self.processFaceNode.send_signal(signal.SIGTERM)
            cmd="rosnode kill /qbo_face_tracking"
            subprocess.Popen(cmd.split())
            cmd="rosnode kill /qbo_face_recognition"
            subprocess.Popen(cmd.split())
            cmd="rosnode kill /qbo_face_following"
            subprocess.Popen(cmd.split())
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Face Recognizer Process. The process may not exist: "+str(e))
        rospy.loginfo(" END Stopping Face Nodes")
        return "ok"
    def startRecognizeFace(self):
        rospy.loginfo("Qbo Webi: Started face recognizer mode. Waiting for service...")
        rospy.wait_for_service("/qbo_face_recognition/get_name")
        rospy.loginfo("Qbo Webi: Recognize Face service is ready!")
        recog = rospy.ServiceProxy('/qbo_face_recognition/get_name',GetName)
        response = recog()

        if (response.recognized):
            rospy.loginfo("Qbo Webi: Person recognized ->"+response.name)
            return response.name
        else:
            rospy.loginfo("Qbo Webi: Person seen is unkown!")
            return ""

    def learnFace(self,faceName):
        rospy.loginfo("Qbo Webi: Started face learning mode (Capturing person's images). Waiting for service...")
        rospy.wait_for_service("/qbo_face_recognition/learn_faces")
        rospy.loginfo("Qbo Webi: Service for face learning is ready")
        learn = rospy.ServiceProxy('/qbo_face_recognition/learn_faces',LearnFaces)
        rospy.loginfo("Qbo Webi: Person to learn ->"+faceName)

        if type(faceName) is unicode:
            asciiFaceName = unicodedata.normalize('NFKD', faceName).encode('ascii','ignore')
            resultLearn = learn(asciiFaceName)
        else:
            resultLearn = learn(faceName)


        #asciiFaceName = unicodedata.normalize('NFKD', faceName).encode('ascii','ignore')        
        #resultLearn = learn(asciiFaceName)

        rospy.loginfo("Qbo Webi: Learning result->"+str(resultLearn.learned))
        return str(resultLearn)

    def trainingFaces(self):
        #entrenamos las imagenes capturadas
        rospy.loginfo("Qbo Webi: Starting face training")
        rospy.wait_for_service("/qbo_face_recognition/train")
        rospy.loginfo("Qbo Webi: Face Train service is available")
        training = rospy.ServiceProxy('/qbo_face_recognition/train',Train)
        resultTraining = training()
        rospy.loginfo("Qbo Webi: Response from Face Train Service -> "+str(resultTraining.taught))
        return str(resultTraining.taught)


############ Object Training and Recognition ########################

    def launchObjectNode(self):
    	rospy.loginfo("Qbo Webi: Launching Object Recognizer")
        cmd = "roslaunch qbo_webi qbo_object_recognition_training.launch"
        self.processObjectNode = subprocess.Popen(cmd.split())


    def stopObjectNode(self):
        try:
            rospy.loginfo("Qbo Webi: Killing Object Recognizer nodes")
            #self.processObjectNode.send_signal(signal.SIGTERM)
            cmd="rosnode kill /orbit_client"
            subprocess.Popen(cmd.split())
            cmd="rosnode kill /qbo_self_recognizer"
            subprocess.Popen(cmd.split())
            cmd="rosnode kill /qbo_stereo_selector"
            subprocess.Popen(cmd.split())
        except Exception as e:
            rospy.loginfo("ERROR when trying to kill Object Recognizer Process. The process may not exist: "+str(e))
        rospy.loginfo(" END Stopping Object Nodes")
        return "ok"

    def startRecognizeObject(self):
        
        rospy.loginfo("Qbo Webi: Started object recognizer mode. Waiting for service...")
        rospy.wait_for_service("/qbo_object_recognition/recognize_with_stabilizer")
        rospy.loginfo("Qbo Webi: Recognize service is ready!")
        recog = rospy.ServiceProxy('/qbo_object_recognition/recognize_with_stabilizer',RecognizeObject)
        respuesta = recog()

        if (respuesta.recognized):
            rospy.loginfo("Qbo Webi: Object recognized ->"+respuesta.object_name)
            return respuesta.object_name
        else:
            rospy.loginfo("Qbo Webi: Object seen is unkown!")
            return ""


    def learnObject(self,objectName):
        rospy.loginfo("Qbo Webi: Started object learning mode (Capturing object's images). Waiting for service...")
        rospy.wait_for_service("/qbo_object_recognition/learn")
        rospy.loginfo("Qbo Webi: Service for object learning is ready")
        learn = rospy.ServiceProxy('/qbo_object_recognition/learn',LearnNewObject)
        rospy.loginfo("Qbo Webi: Object to learn ->"+objectName)


        if type(objectName) is unicode:
            asciiObjectName = unicodedata.normalize('NFKD', objectName).encode('ascii','ignore')
            resultLearn = learn(asciiObjectName)
        else:
            resultLearn = learn(objectName)


        rospy.loginfo("Qbo Webi: Learning result->"+str(resultLearn.learned))
        return str(resultLearn)

    def trainingObjects(self):
        #entrenamos las imagenes capturadas
        rospy.loginfo("Qbo Webi: Starting object teaching")
        rospy.wait_for_service("/qbo_object_recognition/teach")
        rospy.loginfo("Qbo Webi: Object teach service is available")
        training = rospy.ServiceProxy('/qbo_object_recognition/teach',Teach)
        resultTraining = training()
        rospy.loginfo("Qbo Webi: Response from Object Teach Service -> "+str(resultTraining.taught))
        return "True"

############ Q.bo Cloud functions ########################
    def send_images_to_cloud(self):
        
        #Get the path of the newly captured object's images
        path=roslib.packages.get_pkg_dir('qbo_object_recognition')+"/objects/new_objects/"
        
        path=path.rstrip('/')

        print "Images to send to Cloud are in path: ",path
        
        files_list=os.listdir(path)
        for object_name in files_list:
            object_path=path+'/'+object_name
            print "Object found in path: ",object_path
            if os.path.isdir(object_path):
                images_list=os.listdir(object_path)
                total_response=True
                for image in images_list:
                    response_data = self.send_image(object_name,object_path+'/'+image)
                    try:
                        response = json.loads(response_data)
                        if response:
                            #borro imagenes
                            print "Image sent"
                        else:
                            total_response=False
                    except Exception, e:
                        print e
                        print 'Something went wrong'
                    print object_name, ' ', image, ' ', response_data
                    #print object_name, ' ', image, ' ', send_image(object_name,object_path+'/'+image)
                    #print send_image(object_name,object_path+'/'+image)
                if total_response:
                    print "Transfer has been successful"



    def send_image(self,object_name,image_path):
        data = {'image_file' : open(image_path,'rb'),
               #'data' : 'algo',
               'data':json.dumps({'new_image':True,'lang':str(self.language['current_language'])}),
           }
        req = urllib2.Request(self.cloudAddress+'/object/'+urllib.quote(object_name)+'/', data, {})
        u = urllib2.urlopen(req)
        #print u.read()
        return u.read()

    def get_version(self):
        data={'data':json.dumps({'lang':str(self.language['current_language'])})}
        try:    
            req = urllib2.Request(self.cloudAddress+'/getVersion/',data,{})
            u = urllib2.urlopen(req)
        except:
            return ""
        
        version = u.read().strip()
        return version

    def download_latest_version(self, path_to_store):
        
        #Empty the folder
        shutil.rmtree(path_to_store)

        try:
            urllib.urlretrieve(self.cloudAddress+"/static/"+self.language["current_language"]+"_training.tgz", "training.tgz")
        except:
            return "fail"
        #descomprimo archivo de reconocimientos nuevo
        tar_file = tarfile.open("training.tgz")
        tar_file.extractall(path=path_to_store)
        tar_file.close()
       
        #Remove the .tgz file 
        os.remove("training.tgz")
        return "ok" 
