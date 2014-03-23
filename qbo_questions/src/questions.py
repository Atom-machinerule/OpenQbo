#!/usr/bin/env python
#
#Copyright (C) 2012-2013 Thecorpora Inc.
#
#This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
#
#This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.


import roslib; roslib.load_manifest('qbo_questions')
import rospy
from qbo_listen.msg import Listened
from qbo_talk.srv import Text2Speach

from qbo_face_msgs.msg import FacePosAndDist
from std_msgs.msg import String

import random
import sys
import os

#Load plugins directory
path = roslib.packages.get_pkg_dir("qbo_questions")
sys.path.append(path+"/src/plugins")


global client_speak
global face_detected
global dialogue
global subscribe
global plugins
global lang

def system_language(data):
    try:
        set_language(data.data)
        rospy.loginfo("Language changed to "+data.data)
    except KeyError:
        rospy.loginfo("Error: Language not recognized("+data.data+")")


def speak_this(text):
    global client_speak
    client_speak(str(text))

def listen_callback(data):
    global face_detected  
    global dialogue
    
    text=""
    sentence = data.msg
    rospy.loginfo("Listened: |"+sentence+"|")
   
    if not face_detected:
       rospy.loginfo("Ignoring last sentece because face was not detected")
       return

    if sentence in dialogue:
        output = dialogue[sentence]
        choice=random.choice(output)
        
        if choice[0]=="$":
            choice=choice.replace("$","")
            choice=choice.lower()
            for plug in plugins:
                try:
                   text=getattr(plug,choice)(sentence,lang)
                   break
                except AttributeError:
                    rospy.loginfo("Attibute "+choice +" could not be found:"+ str(dir(plug)))
        else:
            text=choice
        speak_this(text)

def face_callback(data):
    global face_detected
    face_detected = data.face_detected	


def read_dialogues(filename):
    global dialogue
    dialogue = {}

    f = open(filename)    
    for line in f.readlines():
        try:
            line = line.replace("\n","")
            parts = line.split(">>>")

            dialogue_input = parts[0].upper()
            dialogue_output = parts[1].upper().strip()
        
            # we check wheter the input line alreayd exists, if so, we add to its own list
            if dialogue_input in dialogue:                
                dialogue[dialogue_input].append(dialogue_output)
            else:
                #dialogue_input does not exist
                dialogue[dialogue_input] = [dialogue_output]
        except Exception:
            pass        

    f.close()

def set_language(lang):

    if lang!="es" and lang!="en":
        lang="en"

    global subscribe
    try:
        subscribe.unregister()
        rospy.loginfo("Unregistered from previous language")
    except NameError:
        rospy.loginfo("First language set")


    path = roslib.packages.get_pkg_dir("qbo_questions")
    filename = path+'/config/dialogues_'+lang
    print "Dialogue filename loaded: "+filename
    read_dialogues(filename)
    subscribe=rospy.Subscriber("/listen/"+lang+"_questions", Listened, listen_callback)

def loadPlugins():
    global plugins
    plugins=[]
    path = roslib.packages.get_pkg_dir("qbo_questions")
    for f in os.listdir(path+"/src/plugins/"):
        moduleName, ext = os.path.splitext(f) 
        if ext == '.py' and moduleName!="__init__":
            plugins.append(__import__(moduleName))


def main():
    global client_speak
    global face_detected
    global lang

    #Init ROS
    rospy.init_node('qbo_questions')
    lang = rospy.get_param("/system_lang", "en")

    #Init variable to know if somebody is in front of qbo
    face_detected = False

    #Load plugins
    loadPlugins()

    set_language(lang)
    print "Language loaded: "+lang

    print "Dialog => "+str(dialogue)

    rospy.loginfo("Starting questions node")
 
    client_speak = rospy.ServiceProxy("/qbo_talk/festival_say", Text2Speach)

    #Set Julius
    rospy.Subscriber("/system_lang", String, system_language)

    #For face view
    rospy.Subscriber("/qbo_face_tracking/face_pos_and_dist", FacePosAndDist, face_callback)


    rospy.spin()


if __name__ == '__main__':
    main()

