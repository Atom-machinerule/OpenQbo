#!/usr/bin/python
import roslib; roslib.load_manifest('qbo_expression')
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from qbo_talk.srv import Text2Speach

from lib_qbo_pyarduqbo import qbo_control_client as gcc
from time import sleep
import re
import threading 
import math 
import random

import types

from qbo_arduqbo.msg import Mouth as Mouth_msg

lock = threading.Lock()
global_numCmd = 0  #Used when user want to wait to all moves to finish

###### Mouths functions #######
def numberToMouthArray(shapeNumber):
    #create shape array from string number
    #shapeNumber = int(number)
    shapeBinString="{0:b}".format(shapeNumber)
    shape = [ 1 if n=='1' else 0 for n in shapeBinString ]
    while len(shape)<20:
        shape = [0] + shape
    shape.reverse()
    return shape

class Mouth:
    def __init__(self,idN,name,shape):
        if type(shape)==types.IntType:
            shape=numberToMouthArray(shape)
        self.idN=int(idN)
        self.name=str(name)
        self.shape=shape

class mouth:

    def __init__(self):
	self.mouth_pub=rospy.Publisher('/cmd_mouth', Mouth_msg)
        self.vMouths=[]

        self.vMouths.append(Mouth(0,"Happy",476160,));
        self.vMouths.append(Mouth(1,"Sad",571392));
        self.vMouths.append(Mouth(2,"Ooh",476718));
        self.vMouths.append(Mouth(3,"Pucker the mouth to the right",17376));
        self.vMouths.append(Mouth(4,"Pucker the mouth to the left",2016));
        self.vMouths.append(Mouth(5,"Straight face",31744));
        self.vMouths.append(Mouth(6,"Small mouth",141636));
        self.vMouths.append(Mouth(7,"Speak 1",31));
        self.vMouths.append(Mouth(8,"Speak 2",479));
        self.vMouths.append(Mouth(9,"Speak 3",14911));
        self.vMouths.append(Mouth(10,"Speak 4",15359));
        self.vMouths.append(Mouth(11,"Speak 5",476735));
        self.vMouths.append(Mouth(12,"Speak 6",491519));
        self.vMouths.append(Mouth(13,"None",0));

        self.vMouths.append(Mouth(14,"surprise",476718));
        self.vMouths.append(Mouth(15,"regular",69904));
        self.vMouths.append(Mouth(16,"tongue",283616));

    def changeMouth(self,idx):
        boca=Mouth_msg()
        boca.mouthImage=self.vMouths[idx].shape
        self.mouth_pub.publish(boca)


###### HEAD MOVES ######
class ThreadMoveHead(threading.Thread):
    def __init__(self):
            self.alive = False
            self.killMyself = False
            
            self.pan_pos = 0
            self.tilt_pos = 0
            self.pan_speed = 0
            self.tilt_speed = 0

            self.sub = rospy.Subscriber('/joint_states', JointState, self.jointsCb, queue_size=1)
            self.lock = threading.RLock()
           
            self.actualPosX = -1000
            self.actualPosY = -1000
           
	    ordenEjecutada = False
           
    def jointsCb(self,msg):
            with self.lock:
              for i in range(0,len(msg.name)):
                if msg.name[i]=='head_pan_joint':
                    self.actualPosX=math.degrees(msg.position[i])
                    
                elif msg.name[i]=='head_tilt_joint':
                    self.actualPosY=math.degrees(msg.position[i])
              
    def run(self):
            control = gcc()
  
            with self.lock:
              initPosX = self.actualPosX
              initPosY = self.actualPosY
                
            lastPosX = -1000
            lastPosY = -1000            
            
            cont = 0
            contLoop = 0 
            
	    while True and not self.killMyself:
                with self.lock:
                        if self.pan_pos == -1000:
                                self.alive=False
                                self.killMyself = True
                        else:                           
                                          
                            self.alive = True

                            #We check whether the movement have been finished or not, so if the last five movements
                            # are the same, is because it reached the end point.
                            if lastPosX == self.actualPosX and lastPosY == self.actualPosY:
                                    cont +=1
                                    if cont >= 5:
                                            self.pan_pos = -1000
                                            contLoop = 0
                                            cont = 0                           

                            if not self.ordenEjecutada:
				
			       self.ordenEjecutada = True	

                               move = {}
                               move["pan_position"]=self.pan_pos
                               move["pan_speed"]=self.pan_speed
                               move["tilt_position"] = self.tilt_pos
                               move["tilt_speed"] = self.tilt_speed
              
                               control.headServosPut(move)                              


                            lastPosX = self.actualPosX
                            lastPosY = self.actualPosY                              

                sleep(0.1)

            self.alive = False
            self.killMyself = False


    def imBack(self):
        threading.Thread.__init__(self)

    def is_alive(self): 
        return self.alive

    def die(self):
        with self.lock:
                self.killMyself = True
                self.sub.unregister()

    def set(self,pan_pos, pan_speed, tilt_pos, tilt_speed):
        with self.lock:
		self.ordenEjecutada = False
                self.pan_pos = pan_pos
                self.pan_speed = pan_speed
                self.tilt_pos = tilt_pos
                self.tilt_speed = tilt_speed
                
    
###### LEFT Eye Moves ######    
class ThreadMoveLeftEye(threading.Thread):
    def __init__(self):
            self.alive = False
            self.killMyself = False
            
            self.angle = 0 
	    self.newAngle = self.angle           
	    self.initAngle = -1000
            self.actualAngle = -1000            

            self.lock = threading.RLock()

            self.sub = rospy.Subscriber('/joint_states', JointState, self.jointsCb, queue_size=1)
            
           
    def jointsCb(self,msg):
            with self.lock:
              for i in range(0,len(msg.name)):
                if msg.name[i]=='left_eyelid_joint':
                    self.actualAngle=math.degrees(msg.position[i])+90                    
		    if self.initAngle == -1000:
			self.initAngle = self.actualAngle
               
              
    def run(self):
            control = gcc() 
            lastAngle = -1000
            cont = 0
            contLoop = 0 

            while True and not self.killMyself:
                with self.lock:                        
			self.alive = True
			if self.newAngle != self.angle:
				self.angle = self.newAngle
				self.initAngle = self.actualAngle

			if self.actualAngle == self.initAngle:
				control.moveLeftEyelid(self.angle)

                        #If we already are where we wanted, done
                        if round(self.actualAngle) == self.angle:
                                break  
                                                         
			#Sometimes it never ends this loop, this is a security loop breaker
			if contLoop == 50:
				break

			contLoop+=1

                sleep(0.1)

            self.alive = False
            self.killMyself = False


    def imBack(self):
        threading.Thread.__init__(self)

    def is_alive(self): 
        return self.alive

    def die(self):
        with self.lock:
                self.killMyself = True
                self.sub.unregister()


    def set(self,angle):       
        with self.lock:               
		self.newAngle = angle
    

###### RIGHT Eye Moves ######       
class ThreadMoveRightEye(threading.Thread):
    def __init__(self):
            self.alive = False
            self.killMyself = False

            self.angle = 0
            self.newAngle = self.angle
            self.initAngle = -1000
            self.actualAngle = -1000

            self.lock = threading.RLock()

            self.sub = rospy.Subscriber('/joint_states', JointState, self.jointsCb, queue_size=1)

    def jointsCb(self,msg):
            with self.lock:
              for i in range(0,len(msg.name)):
                if msg.name[i]=='right_eyelid_joint':
                    self.actualAngle=math.degrees(msg.position[i])+90
                    if self.initAngle == -1000:
                        self.initAngle = self.actualAngle

    def run(self):
            control = gcc()
	    contLoop = 0

            while True and not self.killMyself:
                with self.lock:
			self.alive = True
                        if self.newAngle != self.angle:
                                self.angle = self.newAngle
                                self.initAngle = self.actualAngle

                        if self.actualAngle == self.initAngle:
                                control.moveRightEyelid(self.angle)

                        #If we already are where we wanted, done
                        if round(self.actualAngle) == self.angle:
                                break

	                #Sometimes it never ends this loop, this is a security loop breaker
                        if contLoop == 50:
                                break
                        contLoop+=1

                sleep(0.1)

            self.alive = False
            self.killMyself = False

    def imBack(self):
        threading.Thread.__init__(self)

    def is_alive(self):
        return self.alive

    def die(self):
        with self.lock:
                self.killMyself = True
                self.sub.unregister()


    def set(self,angle):
        with self.lock:
                self.newAngle = angle


###### Eye Moves ######    
class ThreadMoveEyes(threading.Thread):
    def __init__(self):
            self.alive = False
            self.killMyself = False

            self.angle = 0
            self.newAngle = self.angle
            self.initAngle = -1000
            self.actualAngle = -1000

            self.lock = threading.RLock()

            self.sub = rospy.Subscriber('/joint_states', JointState, self.jointsCb, queue_size=1)


    def jointsCb(self,msg):
            with self.lock:
              for i in range(0,len(msg.name)):
		#As we assume both eyes are moving at the same time, we only take information from one
                if msg.name[i]=='right_eyelid_joint':
                    self.actualAngle=math.degrees(msg.position[i])+90
                    if self.initAngle == -1000:
                        self.initAngle = self.actualAngle


    def run(self):
            control = gcc()
	    securityLoop=200
	    contLoop=0 #sometimes we get an infinite loop, sometimes. Still need to be solved

            while True and not self.killMyself:
                with self.lock:
                        self.alive = True
                        if self.newAngle != self.angle:
                                self.angle = self.newAngle
                                self.initAngle = self.actualAngle

                        if self.actualAngle == self.initAngle:
                                control.movePairEyelid(self.angle)
			
			contLoop += 1
			if contLoop > securityLoop:
				break


                        if round(self.actualAngle) == self.angle:
                                break
                sleep(0.1)

            self.alive = False
            self.killMyself = False

    def imBack(self):
        threading.Thread.__init__(self)

    def is_alive(self):
        return self.alive

    def die(self):
        with self.lock:
                self.killMyself = True
                self.sub.unregister()


    def set(self,angle):
        with self.lock:
                self.newAngle = angle
    
    
###### RELATIVE HEAD MOVES ######
class ThreadMoveRelativeHead(threading.Thread):
    def __init__(self):
            self.alive = False
            self.killMyself = False
            
            self.pan_pos = 0
            self.tilt_pos = 0
            self.pan_speed = 0
            self.tilt_speed = 0

            self.sub = rospy.Subscriber('/joint_states', JointState, self.jointsCb, queue_size=1)
            self.lock = threading.RLock()
           
            self.actualPosX = -1000
            self.actualPosY = -1000
            
            self.originalPosX = -1000
            self.originalPosY = -1000
           
    def jointsCb(self,msg):
            with self.lock:
              for i in range(0,len(msg.name)):
                if msg.name[i]=='head_pan_joint':
                    self.actualPosX=math.degrees(msg.position[i])
                    
                    #We get the initial position as a reference point
                    if self.originalPosX == -1000:
                      self.originalPosX = math.degrees(msg.position[i])
                    
                elif msg.name[i]=='head_tilt_joint':
                    self.actualPosY=math.degrees(msg.position[i])
                     
                    #We get the initial position as a reference point
                    if self.originalPosY == -1000:
                      self.originalPosY = math.degrees(msg.position[i])

              
    def run(self):
            control = gcc()
  
            with self.lock:
              initPosX = self.actualPosX
              initPosY = self.actualPosY
                
            lastPosX = -1000
            lastPosY = -1000            
            
            cont = 0

            while True and not self.killMyself:
                with self.lock:
                        if self.pan_pos == -1000:
                                self.alive=False
                                self.killMyself = True
                        else:                           
                                          
                            self.alive = True

                            #We check whether the movement have been finished or not, so if the last five movements
                            # are the same, is because it reached the end point.
                            if lastPosX == self.actualPosX and lastPosY == self.actualPosY:
                                    cont +=1
                                    if cont >= 5:
                                            self.pan_pos = -1000                                            
                                            cont = 0                           

                            if self.pan_pos != -1000:
                              move = {}
                              move["pan_position"]=self.pan_pos
                              move["pan_speed"]=self.pan_speed
                              move["tilt_position"] = self.tilt_pos
                              move["tilt_speed"] = self.tilt_speed
      
                              control.headServosPut(move)                              
                              lastPosX = self.actualPosX
                              lastPosY = self.actualPosY                              

                sleep(0.1)


            self.alive = False
            self.killMyself = False           


    def imBack(self):
        threading.Thread.__init__(self)

    def is_alive(self): 
        return self.alive

    def die(self):
        with self.lock:
                self.killMyself = True
                self.sub.unregister()

    def set(self,pan_pos, pan_speed, tilt_pos, tilt_speed):
        #We wait for the original values from the head
        while self.originalPosX == -1000 and self.originalPosY == -1000:
          sleep(0.1)
          
        with self.lock:          
                self.pan_pos = (-1*float(self.originalPosX))-float(pan_pos)

                self.pan_speed = pan_speed

                self.tilt_pos = (-1*float(self.originalPosY))-float(tilt_pos)

                self.tilt_speed = tilt_speed
    

###### SAY ######       
class Speak(threading.Thread):
    def __init__(self,sentence):
            threading.Thread.__init__(self)                  
	    self.sentence = sentence
	    self.alive = False

    def run(self):
	    self.alive = True
                  
	    rospy.wait_for_service('/qbo_talk/festival_say')
	    festival = rospy.ServiceProxy('/qbo_talk/festival_say',Text2Speach)

	    r = festival(self.sentence) 
	    self.alive = False


    def is_alive(self): 
	    return self.alive

  
  
class MainThread(threading.Thread):
    def __init__(self,cmdList):
      threading.Thread.__init__(self)
      self.cmdList = cmdList           
      m = mouth()
      m.changeMouth(0)
	

    def run(self):  
      headThread = None
      leftEyeThread = None
      rightEyeThread = None
      relativeHeadThread = None
      EyesThread = None
      m = mouth()

      control = gcc()

      for move in self.cmdList:
          runningThreads = []

          moveList = move.split(" ")

          relativeHeadThread = None

          for i in range(0,len(moveList)-1):              
              if moveList[i]=="l": # LEFT EYE
                  angle = self.getNumber(moveList[i+1])

                  if leftEyeThread == None:
                      leftEyeThread = ThreadMoveLeftEye()

                  leftEyeThread.set(angle)
                  runningThreads.append(leftEyeThread)

		  if not leftEyeThread.is_alive():
	                leftEyeThread.imBack()
        		leftEyeThread.start()

                  i += 2
              elif moveList[i]=="r": # RIGHT EYE
                  angle = self.getNumber(moveList[i+1])

                  if rightEyeThread == None:
                      rightEyeThread = ThreadMoveRightEye()

                  rightEyeThread.set(angle)
                  runningThreads.append(rightEyeThread)


		  if not rightEyeThread.is_alive():
                        rightEyeThread.imBack()
                        rightEyeThread.start()


                  i += 2
              elif moveList[i]=="e": # BOTH EYES
                  angle = self.getNumber(moveList[i+1])

                  if EyesThread == None:
                      EyesThread = ThreadMoveEyes()

                  EyesThread.set(angle)
                  runningThreads.append(EyesThread)


                  if not EyesThread.is_alive():
                        EyesThread.imBack()
                        EyesThread.start()

                  i += 2

              elif moveList[i]=="h": # HEAD
                  pan_pos = self.getNumber(moveList[i+1])
                  tilt_pos = self.getNumber(moveList[i+2])
                  pan_speed = self.getNumber(moveList[i+3])
                  tilt_speed = self.getNumber(moveList[i+4])

                  if headThread == None:
                      headThread = ThreadMoveHead()

                  headThread.set(pan_pos,pan_speed,tilt_pos,tilt_speed)
                  runningThreads.append(headThread)

                  if not headThread.is_alive():
                    headThread.imBack()
                    headThread.start()

                  i +=5
              elif moveList[i]=="v": # RELATIVE HEAD
                  pan_pos = self.getNumber(moveList[i+1])
                  tilt_pos = self.getNumber(moveList[i+2])
                  pan_speed = self.getNumber(moveList[i+3])
                  tilt_speed = self.getNumber(moveList[i+4])

                  if relativeHeadThread == None:
                      relativeHeadThread = ThreadMoveRelativeHead()
                      

                  relativeHeadThread.set(pan_pos,pan_speed,tilt_pos,tilt_speed)
                  runningThreads.append(relativeHeadThread)

                  if not relativeHeadThread.is_alive():
                    relativeHeadThread.imBack()
                    relativeHeadThread.start()

                  i +=5
              elif moveList[i]=="w": # WAIT
                  time = self.getNumber(moveList[i+1])

                  if str(time) == "-1":
                      for thread in runningThreads:
                          thread.join()

                      runningThreads=[]

                  else:
                       sleep( time )

                  i+=2
              elif moveList[i]=="s": # SPEAK
		  sentence = ""  
		  i+=1

		  while True:
			w = moveList[i]
			if w.endswith("]"): 
				w = w.replace("]","")
				sentence = sentence+" "+w
				break				

			sentence = sentence+" "+w
			i+=1

                  say = Speak(sentence)
		  say.start() 
		  runningThreads.append(say)

                  i+=2

              elif moveList[i]=="x": # OTHER FACE
		  expression = moveList[i+1]

		  newExp = MainThread(expression)		
		  newExp.start()

                  i+=2
              elif moveList[i]=="o": # MOUTH
                  if moveList[i+1] == 'smile':                       
                      a = m.changeMouth(0)
                  elif moveList[i+1] == 'none':
                      a = m.changeMouth(13)        
                  elif moveList[i+1] == 'sad':
                      a = m.changeMouth(1)
                  elif moveList[i+1] == 'puckerRight':
                      a = m.changeMouth(3)
                  elif moveList[i+1] == 'puckerLeft':
                      a = m.changeMouth(4)
                  elif moveList[i+1] == 'straight':
                      a = m.changeMouth(5)
                  elif moveList[i+1] == 'smallMouth':
                      a = m.changeMouth(6)
                  elif moveList[i+1] == 'surprise':
                      a = m.changeMouth(14)
                  elif moveList[i+1] == 'regular':
                      a = m.changeMouth(15)
                  elif moveList[i+1] == 'laugh':
                      a = m.changeMouth(7)
                  elif moveList[i+1] == 'laugh2':
                      a = m.changeMouth(8)
                  elif moveList[i+1] == 'laugh3':
                      a = m.changeMouth(9)
                  elif moveList[i+1] == 'laugh4':
                      a = m.changeMouth(10)
                  elif moveList[i+1] == 'laugh5':
                      a = m.changeMouth(11)
                  elif moveList[i+1] == 'tongue':
                      a = m.changeMouth(16)
                  elif moveList[i+1] == 'laugh6':
                      a = m.changeMouth(12)

                  i+=2

          if relativeHeadThread != None:
              relativeHeadThread.die()
          


      if headThread != None:
          headThread.die()
      if rightEyeThread != None:
          rightEyeThread.die()
      if leftEyeThread != None:
          leftEyeThread.die()


    # Some values can be just a number, or a random one between an interval, in which case here we give back
    # that random number
    def getNumber(self,num):
      if str(num).startswith("rand"):
        line = str(num).replace("rand","")
        line = line.replace("(","")
        line = line.replace(")","")
        line = line.replace("&"," ")

        nums = line.split(" ")


        if "." in str(nums[0] or "." in str(nums[1])):
            return random.uniform(float(nums[0]), float(nums[1]))
        else:
            return random.randrange(int(nums[0]), int(nums[1]))

      else:
        if "." in str(num):
            return float(num)
        else:
            return int(num)
  
  
  
                
                
