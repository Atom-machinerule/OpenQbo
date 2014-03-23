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
# Authors: Sergio Merino <s.merino@openqbo.com>;


import roslib; roslib.load_manifest('qbo_listen')
from qbo_listen.srv import spell_word
from qbo_listen.srv import new_word
from qbo_listen.srv import train_word
from qbo_listen.srv import free_train
from lib_qbo_pyarduqbo import qbo_control_client

import rospy
import os
import time
import gen_grammar
import shlex, subprocess
import shutil
import pyJulius
import re

from qbo_talk.srv import Text2Speach
from qbo_listen.msg import Listened
from qbo_arduqbo.msg import LCD
from std_msgs.msg import String
from random import choice

import urllib
import urllib2_file
import urllib2
import sys, json

rospath=os.getenv("QBO_ROS_PACKAGE_PATH")

nato=['ALPHA', 'BRAVO', 'CHARLIE', 'DELTA', 'ECHO', 'FOXTROT', 'GOLF', 'HOTEL', 'INDIA', 'JULIET', 'KILO', 'LIMA', 'MIKE', 'NOVEMBER', 'OSCAR', 'PAPA', 'QUEBEC', 'ROMEO', 'SIERRA', 'TANGO', 'UNIFORM', 'VICTOR', 'WHISKEY', 'XRAY', 'YANKEE', 'ZULU']
#Codigo adaptado del algoritmo de Miguel fileUploadTest.
#
#Urllib2_file.py ha de estar en la misma carpeta

#---------------------------------------------

def send_image(object_name,image_path):
    data = {'image_file' : open(image_path,'rb'), #'data' : 'algo',
            'data':json.dumps({'new_image':True}),
           }
    req = urllib2.Request('http://maps:8800/object/'+urllib.quote(object_name)+'/', data, {})
    u = urllib2.urlopen(req)
    #print u.read()
    return u.read()

def retrain_server():
    data = {'data':json.dumps({'update':True})}
    req = urllib2.Request('http://maps:8800/object/train/', data, {})
    u = urllib2.urlopen(req)
    return u.read()

def send_files_to_server(path):
    path=path.rstrip('/')
    files_list=os.listdir(path)
    for object_name in files_list:
        object_path=path+'/'+object_name
        if os.path.isdir(object_path):
            images_list=os.listdir(object_path)
            total_response=True
            for image in images_list:
                response_data = send_image(object_name,object_path+'/'+image)
                try:
                    response = json.loads(response_data)
                    if response:
                        #borro imagenes
                        os.remove(object_path+'/'+image)
                    else:
                        total_response=False
                except Exception, e:
                    print e
                    print 'algo malo'
                print object_name, ' ', image, ' ', response_data
                #print object_name, ' ', image, ' ', send_image(object_name,object_path+'/'+image)
                #print send_image(object_name,object_path+'/'+image)
            if total_response:
                os.removedirs(object_path)


def send_WAV(pathWav, transcripcion, url):

        #transcripcion, no solo lleva la transcripcion propiamente dicha, sino tambien el nombre
        #del usuario. Esto es asi pq se usa como nombre de fichero
    print "transcripcion"+transcripcion
    print "pathWav"+pathWav
    print "url "+url
    transcripcion = transcripcion.replace("_"," ")
    print "new trans "+transcripcion
    data = {'wav' : open(pathWav,'rb'),
               #'data' : 'algo',
                'data':json.dumps({'transcripcion':transcripcion}),
        }
    words = transcripcion.split(".")
    print "--------"+words[0]
    print "--------"+words[1]
        #words[0] tendra el nombre del usuario de drupal
        #words[1] la transcripcion del audio

    req = urllib2.Request(url+urllib.quote(words[0])+'/', data, {})
    u = urllib2.urlopen(req)
        #print u.read()
    return u.read()

def speak_this(text):
    global client_speak
    client_speak(str(text))

def spell_callback(data):
    global listened
    global chatsubscribe
    listened = data.msg
    rospy.loginfo("Listened: |"+listened+"|")
    if listened!="":
        chatsubscribe.unregister()

def accept_callback(data):
    global listened
    global chatsubscribe
    tmplistened = data.msg
    rospy.loginfo("Listened: |"+tmplistened+"|")
    if tmplistened=="YES I DID" or tmplistened=="NO I DID NOT" or tmplistened=="YES" or tmplistened=="NO":
        listened=tmplistened
        chatsubscribe.unregister()

def getALPHANATO(letter):
    for word in nato:
        if word[0]==letter:
            result=" " + letter+" of "+word
            break
    return result

def waitaccept():
    global listened
    global chatsubscribe
    listened=""
    chatsubscribe=rospy.Subscriber("/listen/en_add_word", Listened, accept_callback)
    while listened==""  and not rospy.is_shutdown():
        time.sleep(0.5)
        rospy.loginfo("Waiting confirmation")
    if listened=="YES I DID" or listened=="YES":
        result=1
    elif listened=="CANCEL":
        result=-1
    else:
        result=0
    return result

#def getNumberofletters():
#    global listened
#    global chatsubscribe
#    end=False
#    while end==False:
#        listened=""
#        client_speak("Wich is the number of letters of the word?")
#        chatsubscribe=rospy.Subscriber("/Qbo/chatter", Listened, numbers_callback)
#        while listened=="" and not rospy.is_shutdown():
#            time.sleep(0.5)
#            rospy.loginfo("Waiting for number")
#        result=listened
#    client_speak("Did you say, "+result+ "?")
#        response=waitaccept()
#        if response==1:
#        end=True
#    elif response==-1:
#            end=True
#            result=-1
#    return result

def getWord():
    global listened
    global chatsubscribe
    cont_spell=["Go on", "Continue please", "please continue", "you can tell me the next letter", "tell me the next letter please", "What's the next letter?"]
    init_cont_spell=["i got it", "ok"]
    word=""
    winput=""
    client_speak("Please spell the word")
    while not rospy.is_shutdown():
        listened=""
        rospy.loginfo("Starting spelling")
        chatsubscribe=rospy.Subscriber("/listen/en_abcd", Listened, spell_callback)
        while listened=="" and not rospy.is_shutdown():
            time.sleep(0.5)
            rospy.loginfo("Waiting for spelling")
        winput=listened
        if winput=="THAT'S IT":
            word=word.lower()
            rospy.loginfo("The word I understood is. "+word+". Is that correct?")
            client_speak("The word I understood is. "+word+". Is that correct?")
            response=waitaccept()
            if response==1:
                break 
            elif response==0:
                word=""
                client_speak("Please, repeat the word spelling")
        elif winput=="REPEAT" or winput=="REPEAT LAST ONE" or winput=="LET ME REPEAT LAST LET R":
            word=word[:-1]
            if word!="":
                client_speak("The word until now is. "+word)
            else:
                client_speak("Now there's no letters.")
        elif winput=="ABORT SPELLING":
            word=""
            break
        else:
            word=word+winput[0]
            rospy.loginfo("Word until now: |"+word+"|")
            client_speak(choice(init_cont_spell)+" letter " + getALPHANATO(winput[0])+". "+choice(cont_spell))
    rospy.loginfo("Word spelling finishes:"+word)
    word=word.upper()
    return word

def wordexist(word):
    f = open("tmp","w")
    f.write("[sentence]\n")
    f.write(word)
    f.close()
    vError=gen_grammar.verrors("tmp","/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/AM/en/phonems")
    if vError==0:
        gengrammar.createvoca("tmp","/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/AM/en/phonems","tmp1")
        vError=gen_grammar.perrors("tmp1.voca","/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/AM/en/tiedlist")
    print vError
    return vError   

def train(word):
    global chatsubscribe
    global listened
    global client_lcd
    
    newsentence=1
    while newsentence==1:
        client_speak("Now I will say to you some random words and you have to repeat them")
        client_speak("the words also will be displayed in the LCD")
        words=gengrammar.getRandomwords("/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/AM/en/phonems",2)
        text=""
        for i in range(0,2):
            text=text+" "+words[i]
        text=text+" "+word
        words=gengrammar.getRandomwords("/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/AM/en/phonems",2)
        for i in range(0,2):
            text=text+" "+words[i]
        rospy.loginfo(text)
        client_lcd.publish(LCD(rospy.Header(None,None,"frame"),chr(12)+text))
        client_speak(text)
        client_speak("Do you want to change the words")
        if waitaccept()==1:
            continue
        else:
            newsentence=0
        return text


def listen(text):
    global listened
    ok=0
#    pygame.init()
    while ok==0:
        rospy.loginfo("listening")
        try:
            shutil.rmtree("/tmp/record")
        except:
            print "Directory doesnt exist"
        os.makedirs("/tmp/record")
        recorded=0
        p=subprocess.Popen(["julius-4.2","-C","/opt/qbo/ros_stacks/qbo_apps/qbo_addword/config/juliuan.conf"])
        client_speak("Now say the words please")
        while recorded==0:
            rospy.loginfo("looking for files")
            for root, dirs,files in os.walk("/tmp/record"):
                rospy.loginfo("file found:"+str(len(files)))
                for filename in files:
                    print filename
                    fields=filename.split(".")
                    print fields
                    print fields[len(fields)-1]
                    if fields[len(fields)-1]=="wav":
                        recorded=1
                        break
                break
#        if len(files)==2:
#            recorded=1
#            break
    
        time.sleep(0.5)
        p.terminate()
        client_speak("This have been recorded")
        time.sleep(0.5)
        rospy.loginfo("/tmp/record/"+files[0])
        subprocess.call(["aplay","/tmp/record/"+files[0]])
        client_speak("Is it correct")
        if waitaccept()==1:
            ok=1
            pathWav="/tmp/record/"+files[0]
            text=text.strip()
            transcripcion="robot."+text.replace(" ","_")
            url="http://maps.local:8800/wav/"
            send_WAV(pathWav, transcripcion, url)    
            client_speak("Thank you. This word will be added in the next acustic update")

#def addword(word,wtype):
#    directory="/opt/qbo/ros_stacks/qbo_apps/qbo_cpplisten/config/LM/en/"
#    if wtype=="OBJECT":
#    filename=directory+"objects/sentences.conf"
#    elif wtype=="NAME":
#    filename=directory+"recognition/sentences.conf"
#    lab="["+wtype+"]"
#    gengrammar.addwordtypelabel(word, filename, lab)



def move_180():
    qbo_controller=qbo_control_client()
    angular_speed=0.8
    lineal_speed=0.0
    turn_time=1.35
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    angular_speed=0.0
    lineal_speed=0.2
    turn_time=1.2
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)
    time.sleep(turn_time)
    angular_speed=0.0
    lineal_speed=0.0
    qbo_controller.setLinearAngular(lineal_speed,angular_speed)


def handle_freetrain(req):
   # return 0
   # global activado
    #print fileLocation
    global client_lcd
    response=0

    while response==0:
        juliusServer=pyJulius.JuliusServer("./configbis/julius.jconf")
        time.sleep(2)
        pattern = re.compile ('\s?sentence1\:\s+(?P<word>(\w+ )+)\s?')
#    pattern = re.compile ('\s?sentence1\:\s+\<s\>\s+(?P<word>(\w+ )+)\s?\<.s\>')
#    pattern = re.compile ('\s?sentence1\:\s+(?P<word>(\w+ )+)\s?')
        print "inicio reconocedor"
        while not rospy.is_shutdown():
            juliusOutput = juliusServer._process.stdout.readline()
	#print juliusOutput
            result=pattern.match(juliusOutput)
            if result:
                print "RECONOCIDO"
                text=result.group('word')
              #pub.publish(text.strip())
                print text
                juliusServer.stop()
                client_lcd.publish(LCD(rospy.Header(None,None,"frame"),chr(12)+text))
		move_180()
                break
        response=waitaccept()
	move_180()
    return 1



def handle_spelling(req):
    print "recived"
    return getWord()

def handle_newword(req):
    if wordexist(req.word)==0:
        rootdirectory="/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/LM/en/"
        directory=rootdirectory+req.grammar+"/sentences.conf"
        lab="["+req.label+"]"
        gengrammar.addwordtypelabel(req.word,directory,lab)
        gengrammar.compilegrammar(req.grammar)
        
        return 1
    else:
        return 0

def handle_trainword(req):
    words = train(req.word)    
    listen(words)
    return 1


def main():
    global client_speak
    global client_lcd
    global language

    language="en"

    rospy.init_node('grammar')

    client_speak = rospy.ServiceProxy("/Qbo/festivalSay", Text2Speach)
    client_lcd = rospy.Publisher('cmd_lcd', LCD)

    s = rospy.Service("/Qbo/grammar/spelling", spell_word, handle_spelling)
    s = rospy.Service("/Qbo/grammar/freetrain", free_train, handle_freetrain)
    s = rospy.Service("/Qbo/grammar/addword", new_word, handle_newword)
    s = rospy.Service("/Qbo/grammar/trainword", train_word, handle_trainword)
    rospy.spin()
    
if __name__ == '__main__':
    main()

