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
# Authors: Arturo Bajuelos <arturo@openqbo.com>, 
#          Sergio Merino <sergio.merino@openqbo.com>


import roslib; roslib.load_manifest('qbo_brain')
import rospy

from qbo_listen.msg import Listened
from qbo_talk.srv import Text2Speach

from sensor_msgs.msg import Image
from qbo_face_msgs.msg import FacePosAndDist
from std_msgs.msg import String

import json
import smach
import smach_ros
import signal
import subprocess
import time
global robot_model

#ROS Publishers
global client_speak
global lang_pub
global face_detected

global lang
global language

global active_check_face_object

def run_process(command = ""):

    if command != "":
        return subprocess.Popen(command.split())
    else:
        return -1

def run_all_process(all_commands):
    proc=[]
    for command in all_commands:
        proc.append(subprocess.Popen(command.split()))
    return proc

def kill_all_process(processes):
    for process in processes:
        process.send_signal(signal.SIGINT)


def speak_this(text):
    global client_speak
    client_speak(str(text))

class RobotModel:
    def __init__(self):
        self.last_object_time = rospy.Time.now()
        self.time_threshold = 0.7
        self.random_move = False
        self.follow_face = False

class CommonQboState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["exit"])
        self.state="none"
        self.input_values={language["STOP STATE MACHINE"]:"exit"}
        self.next_state=""
        self.launchers=[]
        self.subscribe = None

    def listen_callback(self, data):
        sentence = data.msg
        rospy.loginfo("Listened: |"+sentence+"|")
        global robot_model
        global face_detected
        global language
        global lang
        global lang_pub

        if self.state=="Default" and sentence == language["HALT YOU ARE MOVE"]:
                
                if robot_model.random_move:
                        run_process("rosnode kill /qbo_random_move")
                        robot_model.random_move = False
                
                rospy.set_param("/qbo_face_following/move_base", False)
                rospy.follow_face = False
                speak_this(language["OK. I STOPPED MOVING"])
                return
 
        if self.state=="Default" and not face_detected:
                print "IGNORING PREVIOUS SENTENCE"
                return
        if self.state=="Default" and sentence == language["WHY DON'T YOU MOVE AROUND"] and not robot_model.random_move:
                run_process("rosrun qbo_random_move qbo_random_move_face_recog.py")
                robot_model.random_move = True
                speak_this(language["OK. I'M TAKING A WALK"])
        
        elif self.state == "Default" and sentence == language["CAN YOU FOLLOW ME"] and not robot_model.follow_face:
                if robot_model.random_move:
                        run_process("rosnode kill /qbo_random_move")
                        robot_model.random_move = False
                
                speak_this(language["YES, I CAN FOLLOW YOU"])
                rospy.follow_face = True
                rospy.set_param("/qbo_face_following/move_base", True)

        elif self.state == "Default" and lang == "en" and sentence == "CAN YOU SPEAK SPANISH":
            #CHANGE LANGUAGE
            self.subscribe.unregister()
            self.subscribe=rospy.Subscriber("/listen/es_default", Listened, self.listen_callback)	
            lang_pub.publish(String("es"))
            loadDictionary("es")
            lang="es"
            rospy.set_param("/system_lang", "es")
            speak_this("SI QUE PUEDO HABLAR ESPAÑOL")
  	
        elif self.state == "Default" and lang == "es" and sentence == "PUEDES HABLAR INGLES":
            #CHANGE LANGUAGE
            self.subscribe.unregister()
            self.subscribe=rospy.Subscriber("/listen/en_default", Listened, self.listen_callback)
            lang_pub.publish(String("en"))
            loadDictionary("en")
            lang="en"
            rospy.set_param("/system_lang", "en")
            speak_this("YES, I CAN SPEAK ENGLISH")        

        try:
            #self.next_state=self.input_values[data.msg]
            lang_label = [key for key, value in language.iteritems() if value == data.msg][0]
            self.next_state=self.input_values[lang_label]
        except:
            rospy.loginfo("Sentence not found")
            
#Define default state
class default(CommonQboState):
    def __init__(self):
        global language
        
        smach.State.__init__(self, outcomes=['mplayer','phone','questions','webi',''])
        self.state="Default"

        self.input_values={"RUN MUSIC PLAYER":"mplayer", "RUN PHONE SERVICES":"phone","RUN WEB INTERFACE":"webi","LAUNCH CHAT MODE":"questions"}
        self.launchers=["roslaunch qbo_brain default_state.launch"]
        #self.launchers=[]
        
    def execute(self, userdata): 
        global lang
        global active_check_face_object

        active_check_face_object = True

        rospy.loginfo('Executing: State '+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)
        
        #Change language to english
#        changeLang = rospy.ServiceProxy("/qbo_talk/festival_language", Text2Speach)
#        changeLang("cmu_us_awb_arctic_clunits")

	#Check if qbo_listen is down
        rosnode_list = runCmdOutput("rosnode list")
        if rosnode_list.find("qbo_listen") == -1:
             run_process("rosnode kill /qbo_audio_control")
             time.sleep(2)
             run_process("roslaunch qbo_listen voice_recognizer.launch")

        #Subscribe to topics
        #Listeners
        self.subscribe=rospy.Subscriber("/listen/"+lang+"_default", Listened, self.listen_callback)
        
        #Stereo Selector
        rospy.Subscriber("/qbo_stereo_selector/object", Image, stereo_selector_callback)    
        #Face Tracking
        rospy.Subscriber("/qbo_face_tracking/face_pos_and_dist", FacePosAndDist, face_pos_callback)
        
        speak_this(language["DEFAULT MODE IS ACTIVE"])

        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
        

        if not rospy.is_shutdown():
            speak_this(language["EXITING DEFAULT MODE"])
            self.subscribe.unregister()
            rospy.loginfo("NextState: "+self.next_state)
  
        active_check_face_object = False
        kill_all_process(pids)
        return self.next_state

class questions(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop',''])
        self.state="Questions"
        self.input_values={"STOP CHAT MODE":"stop","STOP STATE MACHINE":"exit"}
        self.launchers=["roslaunch qbo_questions qbo_questions.launch"]
        #self.launchers=[]

    def execute(self, userdata):
        global lang
        rospy.loginfo('Executing: State '+self.state)
        self.next_state=""
        
        speak_this(language["CHAT MODE IS ACTIVE"])
	time.sleep(4)
	pids=run_all_process(self.launchers)

        #Subscribe to topics
        #Listeners
        self.subscribe=rospy.Subscriber("/listen/"+lang+"_default", Listened, self.listen_callback)

        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")

        if not rospy.is_shutdown():
            speak_this(language["EXITING CHAT MODE"])
            self.subscribe.unregister()
            rospy.loginfo("NextState: "+self.next_state)
 
        kill_all_process(pids)
        time.sleep(5.0)
        return self.next_state

class webi(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop',''])
        self.state="Webi"
        self.input_values={"STOP WEB INTERFACE":"stop", "STOP STATE MACHINE":"exit"}
        self.launchers=["roslaunch qbo_webi qbo_webi.launch"]
        #self.launchers=[]

    def execute(self, userdata):
        global lang
        rospy.loginfo('Executing: State '+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)

        #Subscribe to topics
        #Listeners
        self.subscribe=rospy.Subscriber("/listen/"+lang+"_default", Listened, self.listen_callback)

        speak_this(language["QBO WEB INTERFACE IS ACTIVE"])

        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
        


        if not rospy.is_shutdown():
            changeLang = rospy.ServiceProxy("/qbo_talk/festival_language", Text2Speach)
	    changeLang("cmu_us_awb_arctic_clunits")
            speak_this(language["STOPPING QBO WEB INTERFACE"])
            self.subscribe.unregister()
            rospy.loginfo("NextState: "+self.next_state)

        kill_all_process(pids)

        time.sleep(5.0)
        return self.next_state

class musicplayer(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','phone', ''])
        self.state="Music Player"
        self.input_values={"STOP MUSIC PLAYER":"stop", "START PHONE SERVICES":"phone", "STOP STATE MACHINE":"exit"}
        self.launchers=["roslaunch qbo_music_player hand_gesture_node.launch"]
        
    def execute(self, userdata):
        global lang 
        rospy.loginfo('Executing state'+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)
        self.subscribe=rospy.Subscriber("/listen/"+lang+"_default", Listened, self.listen_callback)

        speak_this(language["MUSIC PLAYER IS ACTIVE"])

        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
                
        if not rospy.is_shutdown():
            speak_this(language["EXITING MUSIC PLAYER"])
            self.subscribe.unregister()
            rospy.loginfo("NextState: "+self.next_state)
        
        kill_all_process(pids)
        return self.next_state
        
class phone(CommonQboState):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop','exit',''])
        self.state="Phone services"
        self.input_values={"STOP PHONE SERVICES":"stop", "STOP STATE MACHINE":"exit"}
        #self.launchers=["roslaunch qbo_http_api_login phone_services.launch"]
        self.launchers=["rosrun qbo_http_api_login qbo_http_api_login.py > /home/qboblue/http_log.txt", "rosrun qbo_mjpeg_server mjpeg_server"]

    def execute(self, userdata): 
        global lang
        rospy.loginfo('Executing state'+self.state)
        self.next_state=""
        pids=run_all_process(self.launchers)
        #run_process("roslaunch qbo_audio_control audio_control_sip.launch")
        self.subscribe=rospy.Subscriber("/listen/"+lang+"_default", Listened, self.listen_callback)
   

        speak_this(language["PHONE SERVICES ARE ACTIVE"])
    
        while self.next_state=="" and not rospy.is_shutdown():
                time.sleep(0.2)
                #rospy.loginfo("Waiting sentence")
                
        speak_this(language["PHONE SERVICES ARE SHUT DOWN"])

	if not rospy.is_shutdown():
            self.subscribe.unregister()
            rospy.loginfo("NextState:"+self.next_state)

        kill_all_process(pids)
        #run_process("roslaunch qbo_audio_control audio_control_listener.launch")
        return self.next_state

def stereo_selector_callback(data):
    global robot_model
    
    robot_model.last_object_time = rospy.Time.now()
    
#    check_face_object_balance()
       
def face_pos_callback(data):
    global robot_model
    global balance_size
    global face_detected

    face_detected = data.face_detected
 
#    check_face_object_balance()
    

def check_face_object_balance(event):
    global robot_model

    global active_check_face_object

    if active_check_face_object == False:
        return  
      
    
    diff_time = rospy.Time.now() - robot_model.last_object_time
   
    '''   
    if rospy.has_param("/qbo_stereo_selector/move_head"):
        object_active = rospy.get_param("/qbo_stereo_selector/move_head")
    else:
        object_active = False

    ''' 
     
    object_active = False
    try: 
       object_active = rospy.get_param("/qbo_stereo_selector/move_head")
    except Exception:
       print "No move head parameter yet"  
    

    if object_active and diff_time.to_sec()>robot_model.time_threshold:
        #Need to activate face mode
        rospy.set_param("/qbo_stereo_selector/move_head", False)
        rospy.set_param("/qbo_face_following/move_head", True)
        print "FACE RECOGNITION MODE"
         
    if (not object_active) and diff_time.to_sec()<robot_model.time_threshold:
        #Need to activate object_mode
        rospy.set_param("/qbo_face_following/move_head", False)
        rospy.set_param("/qbo_stereo_selector/move_head", True)
        print "OBJECT RECOGNITION MODE"

def runCmdOutput(cmd, timeout=None):
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

    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

   
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

  #  print "PID: "+str(p.pid)

    ph_out, ph_err = p.communicate()


    return ph_out

def loadDictionary(lang_sym):
    global language
    
    lang_path=roslib.packages.get_pkg_dir('qbo_brain')+"/config/lang/"
    #Load default dict
    fp=open(lang_path+lang_sym+'.txt','r')
    language=json.load(fp,'utf-8')
    fp.close()

        
def main():
    global client_speak
    global lang_pub
    global robot_model
    global face_detected 
    global language
    global lang
    global active_check_face_object

    active_check_face_object = False

    face_detected = False
   
    rospy.init_node("qbo_brain")
    rospy.loginfo("Starting Qbo Brain")
    
    client_speak = rospy.ServiceProxy("/qbo_talk/festival_say_no_wait", Text2Speach)
    rospy.loginfo("Waiting for the qbo_talk service to be active")
    rospy.wait_for_service('/qbo_talk/festival_say_no_wait')
   

    lang_pub = rospy.Publisher('/system_lang', String)


    lang = 'en'

    if rospy.has_param("/system_lang"):
         lang = rospy.get_param("/system_lang")

    rospy.loginfo("Language active: "+lang)
 
    loadDictionary(lang)

    #Initialize robot model
    robot_model = RobotModel()
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
 

    #Timer for the check_face_object_callback
    rospy.Timer(rospy.Duration(0.2), check_face_object_balance)

    with sm:
        smach.StateMachine.add('default', default(), transitions={'mplayer':'music_player','phone':'phoneserver','webi':'webi','questions':'questions','':'exit'})
        smach.StateMachine.add('music_player', musicplayer(), transitions={'stop':'default', 'phone':'phoneserver', '':'exit'})
        smach.StateMachine.add('phoneserver', phone(), transitions={'stop':'default', '':'exit'})
        smach.StateMachine.add('questions', questions(), transitions={'stop':'default', '':'exit'})
        smach.StateMachine.add('webi', webi(), transitions={'stop':'default', '':'exit'})


    sis= smach_ros.IntrospectionServer('server_name',sm,'/SM_ROOT')
    sis.start()

    speak_this(language["QBO BRAIN IS ACTIVE"])

    # Execute SMACH plan
    rospy.loginfo("State machine launched")
    outcome = sm.execute()
    rospy.loginfo('Finishing state machine')
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()
