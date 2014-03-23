#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012 Thecorpora, Inc.
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

import roslib; roslib.load_manifest('qbo_arduqbo')
import rospy
import sys
import time
import math
from qbo_arduqbo.msg import *
from qbo_arduqbo.srv import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#De este solo habra una instancia en qbo_motion_server.py
#Tendra almacenado el estado del robot
#Proporciona las clases para comunicar con ROS

FRT_OFFSET_Z = 150
BCK_OFFSET_Z = -80

class qbo_control_client():
    def __init__(self):
        #rospy.init_node('qbo_control', anonymous=True)
        self.speed=0
        self.turn=0
        self.realSpeed=0
        self.realTurn=0
        self.speed_step=0.25
        self.turn_step=0.5
        self.pitch=0.0  #en radianes
        self.yaw=0.0    #en radianes
        self.pitch_step=0.1
        self.yaw_step=0.1
        self.mic0=0.0
        self.mic1=0.0
        self.mic2=0.0
        self.Servo1Pos=0
        self.Servo2Pos=0
        self.Servo3Pos=0
        self.Servo4Pos=0
        self.Servo1Speed=0.0
        self.Servo2Speed=0.0
        self.Servo3Speed=0.0
        self.Servo4Speed=0.0
        self.Srf10Distance224=0
        self.Srf10Distance226=0
        self.Srf10Distance228=0
        self.Srf10Distance230=0
        self.Srf10Distance232=0
        self.Srf10Distance234=0
        self.frontalLeftDistance=(0.0,None)
        self.frontalRightDistance=(0.0,None)
        self.bateryStatus=100.0
        self.Irda1Value=0
        self.Irda2Value=0
        self.Irda3Value=0
        self.Motor1Velocidad=0.0
        self.Motor2Velocidad=0.0
        self.mouthValue=0
        self.ActualySpeed=0.0

        self.x=0.0
        self.y=0.0
        self.ang=0.0

        self.cmd_vel_pub=rospy.Publisher('/cmd_vel', Twist)
        self.cmd_joints_pub=rospy.Publisher('/cmd_joints', JointState)
        self.cmd_lcd_pub=rospy.Publisher('/cmd_lcd', LCD)
        
        # subscriptions
        rospy.Subscriber('/joint_states', JointState, self.jointsCb, queue_size=1)
        rospy.Subscriber('/mics_states', NoiseLevels, self.micsCb, queue_size=1)
        rospy.Subscriber('/battery_state', BatteryLevel, self.batteryCb, queue_size=1)
        rospy.Subscriber('/srf10_states', PointCloud, self.srf10Cb, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odometryCb, queue_size=1)
        rospy.Subscriber('/distance_sensors_state/front_right_srf10', PointCloud, self.frontalRightsrf10Cb, queue_size=1)
        rospy.Subscriber('/distance_sensors_state/front_left_srf10', PointCloud, self.frontalLeftsrf10Cb, queue_size=1)

        #test service
        self.test_client = rospy.ServiceProxy('/qbo_arduqbo/test_service', Test)

    def node_info(self):
        nodeInfo={}
        nodeInfo['name']='qbo_arduqbo'
        nodeParams=['speed','position','LCD','SRFs','headServos','eyesServos','MICs','battery','mouth']
        nodeInfo['params']=nodeParams
        nodeInfo['status']='Working'
#Test
    def testBoards(self):
        try:
          testResponse = self.test_client()
          testDic={}
          testDic['SRFcount']=testResponse.SRFcount
          testDic['SRFAddress']=testResponse.SRFAddress
          testDic['Gyroscope']=testResponse.Gyroscope
          testDic['Accelerometer']=testResponse.Accelerometer
          testDic['LCD']=testResponse.LCD
          testDic['Qboard3']=testResponse.Qboard3
          testDic['Qboard1']=testResponse.Qboard1
          testDic['Qboard2']=testResponse.Qboard2
          return testDic
        except Exception, e:
          print 'Error: ',e
          return False

#Sensores
    def ultrasonic_cloud_callback(self,data,points):
        points=[]
        points.append(data.points[0].z)
        points.append(data.points[1].z)
        points.append(data.points[2].z)
        points.append(data.points[3].z)

    def getAllDistances(self):
        return [self.Srf10Distance224,self.Srf10Distance228,self.Srf10Distance230,self.Srf10Distance234]
        #srf10Response={}
        #srf10Response['detected_sensors']=
        #srf10Response['frontal_left']=

    def getFrontalDistances(self):
        return [self.frontalLeftDistance,self.frontalRightDistance]
            
    def srf10Cb(self,msg):
        points=[]
        points.append(msg.points[0].z)
        points.append(msg.points[1].z)
        points.append(msg.points[2].z)
        points.append(msg.points[3].z)
        self.Srf10Distance224=points[0]-FRT_OFFSET_Z
        self.Srf10Distance228=points[1]-FRT_OFFSET_Z
        self.Srf10Distance230=-(points[2]-BCK_OFFSET_Z)
        self.Srf10Distance234=-(points[3]-BCK_OFFSET_Z)

    def frontalRightsrf10Cb(self,msg):
        self.frontalRightDistance=(msg.points[0].z,rospy.Time.now())

    def frontalLeftsrf10Cb(self,msg):
        self.frontalLeftDistance=(msg.points[0].z,rospy.Time.now())

    def getDistance(self,sensor):
        distances=getAllDistances()
        return distances[sensor]

    def getInfrared(self):
        return 0

#Pantalla LCD
    def clearDisplay(self):
        self.cmd_lcd_pub.publish(LCD(msg="%12"))
        return ''

    def setCursor(self,row, col):
        data="%12"+chr(row)+chr(col)
        self.cmd_lcd_pub.publish(data)
        return ''

    def setText(self,text):
        self.cmd_lcd_pub.publish(LCD(msg=text))
        return ''

    def LCDPut(self,params):
        if params.has_key('clear') and params['clear'] is True:
            self.clearDisplay()
        if params.has_key('cursor') and len(params['cursor'])==2:
            self.setCursor(params['cursor'][0],params['cursor'][1])
        if params.has_key('text'):
            self.setText(params['text'])

#Motores
    def sendSpeed(self):
        speed_command=Twist()
        speed_command.linear.x=self.speed;
        speed_command.linear.y=0;
        speed_command.linear.z=0;
        speed_command.angular.x=0;
        speed_command.angular.y=0;
        speed_command.angular.z=self.turn;
        self.cmd_vel_pub.publish(speed_command)

    def stopMotion(self):
        self.speed=0
        self.turn=0
        self.sendSpeed()

    def setLinearAngular(self,linear,angular):
        self.speed=linear
        self.turn=angular
        self.sendSpeed()

    
    def speedPut(self,params):
        if params.has_key('linearSpeed'):
            self.speed=params['linearSpeed']
        if params.has_key('angularSpeed'):
            self.turn=params['angularSpeed']
        if params.has_key('leftBlocked') and params['leftBlocked'] is True:
            #se quita el stall
            pass
        if params.has_key('rightBlocked') and params['rightBlocked'] is True:
            #se quita el stall
            pass
        self.sendSpeed()
        #self.unblockWheels()

    def speedGet(self):
        speedDic={}
        speedDic['leftBlocked']=False
        speedDic['rightBlocked']=False
        speedDic['linearSpeed']=self.realSpeed #De la odometria
        speedDic['angularSpeed']=self.realTurn #De la odometria
        return speedDic

    def getOdometry(self):
        return ''

    def odometryCb(self,msg):
        #print 'llega mensaje de odometria'
        self.x=msg.pose.pose.position.x    
        self.y=msg.pose.pose.position.y
        self.ang=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]
        self.realSpeed=msg.twist.twist.linear.x
        self.realTurn=msg.twist.twist.angular.z

    def positionGet(self):
        positionDic={}
        positionDic['x']=self.x
        positionDic['y']=self.y
        positionDic['ang']=self.ang
        return positionDic

#Boca

#Posicion de los servos

    def jointsCb(self,msg):
        for i in range(0,len(msg.name)):
            if msg.name[i]=='head_pan_joint':
                self.Servo1Pos=math.degrees(msg.position[i])+90
            elif msg.name[i]=='head_tilt_joint':
                self.Servo2Pos=math.degrees(msg.position[i])+90
            elif msg.name[i]=='left_eyelid_joint':
                self.Servo3Pos=math.degrees(msg.position[i])+90
            elif msg.name[i]=='right_eyelid_joint':
                self.Servo4Pos=math.degrees(msg.position[i])+90
            if len(msg.velocity)==len(msg.name):
                if msg.name[i]=='head_pan_joint':
                    self.Servo1Speed=msg.velocity[i]
                elif msg.name[i]=='head_tilt_joint':
                    self.Servo2Speed=msg.velocity[i]
                elif msg.name[i]=='left_eyelid_joint':
                    self.Servo3Speed=msg.velocity[i]
                elif msg.name[i]=='right_eyelid_joint':
                    self.Servo4Speed=msg.velocity[i]

#Cabeza
    def head_pose_callback(self,data, positions):
        positions=[]
        positions.append(data.pitch)
        positions.append(data.yaw)

    def sendHeadPose(self):
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()

        msg.name.append('head_pan_joint')
        msg.position.append(float(self.yaw))
        msg.velocity.append(1.0)
        msg.name.append('head_tilt_joint')
        msg.position.append(float(self.pitch))
        msg.velocity.append(1.0)
  
        msg.header.stamp = rospy.Time.now()


	#print "mensaje en lib_qbo_pyarduqbo:    vamos a mover la cazbeza ",msg
        self.cmd_joints_pub.publish(msg)

    def moveHeadHorizontally(self,value):
        self.yaw=-math.radians(float(value)-float(90))
        self.sendHeadPose()
        return ''

    def moveHeadVertically(self,value):
        grados=(float(value)-float(90))/180
        if grados<0:
            grados=grados*20
        else:
            grados=grados*70
        self.pitch=-math.radians(grados)
        self.sendHeadPose()
        return ''

    def relativeMoveHeadHorizontally(self,value):
        self.yaw+=-math.radians(value)
        self.sendHeadPose()
        return ''

    def relativeMoveHeadVertically(self,value):
        grados=(float(value)-float(90))/180
        if grados<0:
            grados=grados*20
        else:
            grados=grados*70
        self.pitch+=-math.radians(grados)
        self.sendHeadPose()
        return ''

    def moveHead(self,horizontal,vertical):
        self.yaw=-math.radians(float(horizontal))
        grados=(float(vertical))/180
        if grados<0:
            grados=grados*70
        else:
            grados=grados*20
        self.pitch=-math.radians(grados)
        self.sendHeadPose()
        return ''

    def getHeadPosition(self):
        return [self.Servo1Pos,self.Servo2Pos]

    #Revisar este put
    def headServosPut(self,params):
        if params.has_key('pan_position'):
            #self.yaw=-math.radians(float(params['pan_position']))
            grados=80*(float(params['pan_position']))/90
            self.yaw=-math.radians(grados)
        if params.has_key('pan_speed'):
            #Anadir velocuidad
            pass
        if params.has_key('tilt_position'):
            grados=-(float(params['tilt_position']))/90
            if grados<0:
                grados=grados*20
            else:
                grados=grados*40
            self.pitch=-math.radians(grados)
        if params.has_key('tilt_speed'):
            #Anadir velocuidad
            pass
        if params.has_key('panBlocked') and params['panBlocked'] is True:
            #se quita el stall
            pass
        if params.has_key('tiltBlocked') and params['tiltBlocked'] is True:
            #se quita el stall
            pass
        self.sendHeadPose()

    def headServosGet(self):
        headServosDic={}
        headServosDic['pan_position']=self.Servo1Pos
        headServosDic['tilt_position']=self.Servo2Pos
        headServosDic['pan_speed']=self.Servo1Speed
        headServosDic['tilt_speed']=self.Servo2Speed
        headServosDic['panBlocked']=False
        headServosDic['tiltBlocked']=False
        return headServosDic

#Ojos
    def getEyesPositions(self):
        return 0

    def moveLeftEyelid(self,value):
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()

        msg.name.append('left_eyelid_joint')
        if not value: return ''
        try:
          msg.position.append(math.radians(float(value)-float(90)))
        except:
          return ''
        msg.velocity.append(100.0)
  
        msg.header.stamp = rospy.Time.now()
        self.cmd_joints_pub.publish(msg)
        return ''

    def moveRightEyelid(self,value):
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()

        msg.name.append('right_eyelid_joint')
        if not value: return ''
        try:
          msg.position.append(math.radians(float(value)-float(90)))
        except:
          return ''
        msg.velocity.append(100.0)
  
        msg.header.stamp = rospy.Time.now()
        self.cmd_joints_pub.publish(msg)
        return ''

    def movePairEyelid(self,value):
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()
        msg.name.append('right_eyelid_joint')
        msg.name.append('left_eyelid_joint') 
        if not value: return ''
        try:
            msg.position.append(math.radians(float(value)-float(90)))
            msg.position.append(math.radians(float(value)-float(90)))
        except:
            return ''
        msg.velocity.append(3.0)
        msg.velocity.append(3.0)

        msg.header.stamp = rospy.Time.now()
        self.cmd_joints_pub.publish(msg)

    def eyelidServosPut(self,params):
        msg = JointState()
        msg.name = list()
        msg.position = list()
        msg.velocity = list()
        msg.effort = list()
        if params.has_key('leftEyelid_position'):
            msg.name.append('left_eyelid_joint') 
            msg.position.append(math.radians(float(params['leftEyelid_position'])-float(90)))
        if params.has_key('leftEyelid_speed'):
            #Anadir velocuidad
            msg.velocity.append(params['leftEyelid_speed'])
        else:
            msg.velocity.append(3.0)
        if params.has_key('rightEyelid_position'):
            msg.name.append('right_eyelid_joint')
            msg.position.append(math.radians(float(params['rightEyelid_position'])-float(90)))
        if params.has_key('rightEyelid_speed'):
            #Anadir velocuidad
            msg.velocity.append(params['rightEyelid_speed'])
        else:
            msg.velocity.append(3.0)

        msg.header.stamp = rospy.Time.now()
        self.cmd_joints_pub.publish(msg)

    def eyelidServosGet(self):
        eyelidServosDic={}
        eyelidServosDic['leftEyelid_position']=self.Servo3Pos
        eyelidServosDic['rightEyelid_position']=self.Servo4Pos
        eyelidServosDic['leftEyelid_speed']=self.Servo3Speed
        eyelidServosDic['rightEyelid_speed']=self.Servo4Speed
        return eyelidServosDic


#Microfonos
    def getMicValues(self):
        return str(self.mic0) + ' ' + str(self.mic1) + ' ' + str(self.mic2)
            
    def micsCb(self,msg):
        self.mic0=msg.m0
        self.mic1=msg.m1
        self.mic2=msg.m2

    def MICsGet(self):
        micsParamsDic={}
        #micsParamsDic['micSelected']='mic0'
        micsParamsDic['mic0']=self.mic0
        micsParamsDic['mic1']=self.mic1
        micsParamsDic['mic2']=self.mic2
        return micsParamsDic
        

    #def MICsPut(self,params):
        #if params.has_key('micSelected'):
            #if params['micSelected']=='mute':
                #pass
            #if params['micSelected']=='mic0':
                #pass
            #if params['micSelected']=='mic1':
                #pass
            #if params['micSelected']=='mic2':
                #pass
        

#Bateria
    def getBatteryStatus(self):
        return self.bateryStatus
            
    def batteryCb(self,msg):
        self.bateryStatus=msg.level

    def batteryGet(self):
        batteryParamsDic={}
        batteryParamsDic['status']='discharging'
        batteryParamsDic['level']=self.bateryStatus
        batteryParamsDic['type']='LiFePo4'
        return batteryParamsDic


        
