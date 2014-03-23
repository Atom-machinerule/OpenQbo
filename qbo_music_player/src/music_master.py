#!/usr/bin/env python
# coding: utf-8
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2011 Thecorpora, S.L.
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
# Authors: Arturo Bajuelos <arturo@openqbo.com>

import roslib; roslib.load_manifest("qbo_music_player")
import rospy 
from syscall import runCmd
from std_msgs.msg import String
from qbo_talk.srv import Text2Speach
import time
import re
import json


client_speak = None

playing_music = False

music_volume = 80

global language

def main():
    
    global client_speak
    global language
    
    rospy.init_node("music_master")
    print "Starting Master Listener Node"
    
    rospy.Subscriber("/hand_gesture_node/hand_gesture", String, hand_gesture_callback)  
    client_speak = rospy.ServiceProxy("/qbo_talk/festival_say_no_wait", Text2Speach)
    
    runCmd("xmms2-launcher")


    lang = rospy.get_param("/system_lang", "en")

    loadDictionary(lang)

    speak_this(language["I AM READY TO PLAY MUSIC"])
   
    rospy.Subscriber("/system_lang", String, change_lang_callback)
 
    rospy.spin()
    runCmd("nyxmms2 stop")


def hand_gesture_callback(data):
    
    global language
    command = str(data.data)
    

    rospy.loginfo(rospy.get_name()+": I heard %s",command)
    
    global playing_music
    global music_volume
    
     
    if command == "play":
        
        runCmd("nyxmms2 toggle")
        
        if playing_music :
            speak_this(language["MUSIC PAUSE"])
            playing_music = False
        else:
            runCmd("nyxmms2 play")
            song_info = get_song_info_2()
            speak_this(language["PLAYING"]+" "+song_info[1]+" "+language["BY"]+" "+song_info[0])
            playing_music = True
        
    
    elif command == "stop":
        runCmd("nyxmms2 "+command)  
        speak_this(language["MUSIC STOPPED"])
        playing_music = False
    
    elif command == "next" or command == "prev":
        runCmd("nyxmms2 "+command)        
        if playing_music:
            time.sleep(1)
            song_info = get_song_info_2()
            speak_this(language["PLAYING"]+" "+song_info[1]+" "+language["BY"]+" "+song_info[0])
        else:
            time.sleep(1)
            song_info = get_song_info_2()
            speak_this(language["SONG SELECTED"])
	    speak_this(song_info[1]+" "+language["BY"]+" "+song_info[0])
         
    elif command=="volume_up":
        if music_volume<=80:
            music_volume+=20
        runCmd("nyxmms2 server volume "+str(music_volume))
        speak_this(language["VOLUME UP"])
    
    elif command=="volume_down":
        if music_volume>20:
            music_volume-=20
        runCmd("nyxmms2 server volume "+str(music_volume)) 
        speak_this(language["VOLUME DOWN"])

def get_song_info_2():
    (ph_out, ph_err, ph_ret) = runCmd("nyxmms2 list")
    out_str = str(ph_out)
    
    song_info = out_str.split("->")
    song_info = song_info[1]
    song_info = song_info.split("]")
    song_info = song_info[1]
    song_info = song_info.split("(")
    song_info = song_info[0]
    
    song_info = song_info.strip()
    
    song_info = song_info.split(" - ")
    
    print "Artist: "+song_info[0]+ ", Song: "+song_info[1]
    
    return song_info

def speak_this(text):
    global client_speak
    global language

    client_speak(str(text))


def change_lang_callback(data):
    if str(data.data) != "es" and str(data.data)!="en":
        rospy.loginfo("Language not recognized")
        return

    lang = data.data
    rospy.loginfo("Changing language to "+str(data.data))
    loadDictionary(lang)

    
def loadDictionary(lang_sym):
    global language

    lang_path=roslib.packages.get_pkg_dir('qbo_music_player')+"/config/lang/"
    #Load default dict
    fp=open(lang_path+lang_sym+'.txt','r')
    language=json.load(fp,'utf-8')
    fp.close()


if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException: pass

