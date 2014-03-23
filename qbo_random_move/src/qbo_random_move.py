#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2011 Thecorpora, Inc.
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
# Authors: Miguel Angel Julian <miguel.a.j@openqbo.com>

import roslib
roslib.load_manifest('qbo_random_move')

import rospy
import threading

from lib_qbo_pyarduqbo import qbo_control_client
#from lib_qbo_talk_py import qbo_talk_client
from qbo_talk.srv import Text2Speach
from qbo_face_msgs.msg import FacePosAndDist

from time import sleep
from random import choice, uniform
from math import atan, atan2

talking_flag=False

angular_speed=0.4
lineal_speed=0.2

face_stabilizer = 0
stabilizer_max = 10
stabilizer_thr = 7

def say(text):
    global talking_flag
    if not talking_flag:
        talking_flag=True
        festivalCommand = rospy.ServiceProxy('/qbo_talk/festival_say', Text2Speach)
        resp1 = festivalCommand(text)
        #self.qbo_talk_controller.say('sentence':text)
        #print 'termino de hablar'
        talking_flag=False


def face_callback(data):
   global face_stabilizer
   global stabilizer_max
   if data.face_detected==True and face_stabilizer<stabilizer_max: 
       face_stabilizer+=1
   elif data.face_detected==False and face_stabilizer>0:
       face_stabilizer-=1

#   print "Stabilizer: "+str(face_stabilizer)
      



class qbo_random_move():
    def __init__(self):
        self.qbo_controller=qbo_control_client()
        #self.qbo_talk_controller=qbo_talk_client()
        self.sensors_distance=0.215
        self.lineal_speed=0.0
        self.angular_speed=0.0
        self.turn_time=0.1
        self.wall_distance_limit=0.3
        self.last_turn_direction=False #True means left
        self.bad_word_said=False
        self.uniform_lineal_speed_change=0.05
        #self.talking=False
        self.sentences=['Ups','A can not pass though a wall','I will brake this wall. Can anyone put a cannon on me?','Atom, can you help me with this wall?', 'Oh my God. I am gonna crash', 'I shall not pass']
        rospy.Subscriber("/qbo_face_tracking/face_pos_and_size", FacePosAndDist, face_callback)


    def spin(self):
        global talking_flag
        global face_stabilizer
        global stabilizer_thr
			
        while(not rospy.is_shutdown()):
           
            #If a face is spoted, stop base movement
            if face_stabilizer >= stabilizer_thr:
               self.qbo_controller.setLinearAngular(0,0)
               continue

			 #frontal_distances=[(None,None),(None,None)]
            frontal_distances=self.qbo_controller.getFrontalDistances()
            now=rospy.Time.now()
            warning_left_flag=False
            warning_right_flag=False
            if frontal_distances[0][1] and (now-frontal_distances[0][1])<rospy.Duration(0.5):
                warning_left_flag=True
            if frontal_distances[1][1] and (now-frontal_distances[1][1])<rospy.Duration(0.5):
                warning_right_flag=True

            left_distance=999.9
            right_distance=999.9

            if warning_left_flag:
                left_distance=frontal_distances[0][0]
            if warning_right_flag:
                right_distance=frontal_distances[1][0]

            #print frontal_distances

            if not warning_left_flag or not warning_right_flag:
                if left_distance<right_distance:
                    if self.lineal_speed != 0:
                        self.last_turn_direction=False
                    self.lineal_speed=lineal_speed+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)#0.5
                    self.angular_speed=-angular_speed+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)#0.4
                    #self.last_turn_direction=False
                elif left_distance>right_distance:
                    if self.lineal_speed != 0:
                        self.last_turn_direction=True
                    self.lineal_speed=lineal_speed+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)#0.5
                    self.angular_speed=angular_speed+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)#0.4
                    #self.last_turn_direction=True
                else:
                    self.lineal_speed=lineal_speed+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)#0.5
                    self.angular_speed=0.0+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)
                    #bad_word_said=False
            else:
                angle=atan2((left_distance-right_distance),self.sensors_distance)
                #print (left_distance-right_distance), angle
                self.lineal_speed=lineal_speed+uniform(-self.uniform_lineal_speed_change,self.uniform_lineal_speed_change)
                min_distance=min(left_distance,right_distance)#-self.wall_distance_limit
                time_limit=min_distance/self.lineal_speed
                #print time_limit, min_distance
                #check time to 0 limit
                if min_distance<=0 or time_limit==0:
                    self.lineal_speed=0
                    self.angular_speed=angular_speed
                else:
                    self.angular_speed=angle/time_limit

            #print 'left: ',left_distance, ' right: ',right_distance
            if (left_distance<self.wall_distance_limit) and (right_distance<self.wall_distance_limit):
                #print 'left: ',left_distance, ' right: ',right_distance
                self.lineal_speed=0.0
                if self.last_turn_direction:
                    self.angular_speed=angular_speed+0.4#0.6
                else:
                    self.angular_speed=-angular_speed-0.4#0.6
                if not bad_word_said:# and not talking_flag:
                    #talking_flag=True
                    #print 'intento hablar'
                    t = threading.Thread(target=say, args=(choice(self.sentences), )) #[randint(0,len(self.sentences)-1)], ))
                    t.start()
                    bad_word_said=True
                self.turn_time=0.15+uniform(-0.1,0.1)
            else:
                bad_word_said=False
                self.turn_time=0.1

            self.qbo_controller.setLinearAngular(self.lineal_speed,self.angular_speed)
            sleep(self.turn_time)
        



if __name__ == "__main__":
    #try:
        rospy.init_node('qbo_random_move')
        random_controller=qbo_random_move()
        random_controller.spin()
    #except Exception, e:
        #print 'Excepcion: ', e
        #exit()
