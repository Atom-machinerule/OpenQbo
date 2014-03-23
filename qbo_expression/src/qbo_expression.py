#!/usr/bin/python
import roslib; 
roslib.load_manifest('qbo_expression')

import rospy
from std_msgs.msg import String

from time import sleep
import re
import threading
import random
import roslib.packages
 
from ThreadsCmd import *

feelinglist = {}

# Given a defined feeling, we launch it in a thread
def runFeeling(feeling):
    rospy.loginfo("  "+feeling.data)

    global feelinglist

    if not feelinglist.has_key(feeling.data):
        print feeling.data+" is not defined"
	print "feelings available "+str(feelinglist.keys())
        return -1

    vec = feelinglist[feeling.data]

    aux = str(vec).replace(",","")
    aux = str(aux).replace("[","")
    aux = str(aux).replace("'","")
    aux = aux.split("'m'")

    #We call the MainThread defined in ThreadsCmd.py, it will manage all the expression's movements
    thread = MainThread(aux)  
    thread.start()



        
#
# We take the file feelings.xml and transform it into an array 
#
# For example: if in the file we have
#
# <feeling neutral>
#        <movement>
#                <eyes 270>
#                <mouth none>
#                <head 0 50 1.0 1.0>
#                <wait -1>
#        </movement>
# </feeling>
#
#
# this will generate this array:
# {'neutral': ['m', 'e', '270', 'o', 'none', 'h', '0', '50', '1.0', '1.0', 'w', '-1', 'm']}
#
# Which will be interpreted later on by ThreadsCmd.py
#
def loadFeelings():
        lineCount = 0
        cmdExpected = "feeling"
        script = {}
        loop=-1;
        numCmd = 0 #the number of commands inside a movement
        feeling = ""

	path = roslib.packages.get_pkg_dir("qbo_expression")
        f = open(path+"/config/feelings.xml","r")
        for line in f:
                lineCount += 1
                if line == "\n":
                        continue

                line=line.lstrip()
                line=line.replace("<", "")
                line=line.replace(">","")
                line=line.replace("\t","")
                line=line.replace("\n","")                        

                parts = line.split(" ")
                
		if parts[0] == "feeling":
                        if parts[1]:
                                feeling = parts[1]
                                moves = []
                                cmdExpected="movement"
                        else:
                                print "ERROR: feeling label has to get a name. Line "+str(lineCount)
                                return -1

                if parts[0] == "/feeling":
                        script[feeling] = moves
                        cmdExpected="feeling"
			loop = -1

                elif parts[0] == "/movement":
                        #Keep in mind if the last move had a loop			
			try:
	                        if loop != -1:
        	                        mov = moves[len(moves)-numCmd:len(moves)]
                	                for i in range(1,int(loop)):
                        	                moves.extend(mov)

                                	loop = -1;
			except:
				pass

                        moves.append('m')
                        cmdExpected="endFeelingOrMovement"

                elif  parts[0] == "movement":
                        if len(parts) >= 2:
                                loop = parts[1]
                                numCmd=0
			else:
				loop = -1
                        moves.append('m')
                        cmdExpected="cmd"

                elif parts[0] == "leftEye":
                        moves.append('l')
                        if len(parts)>=2:
                                moves.append(parts[1])
                        else:
                                print 'ERROR: you have to add a number for the comand leftEye in line '+str(lineCount)
                                return -1

                        numCmd=numCmd+2
                        cmdExpected="cmdOrEndMovement"

                elif parts[0] == "rightEye":
                        moves.append('r')
                        if len(parts)>=2:
                                moves.append(parts[1])
                        else:
                                print 'ERROR: you have to add a number for the comand rightEye in line '+str(lineCount)
                                return -1

                        numCmd=numCmd+2
                        cmdExpected="cmdOrEndMovement"

		elif parts[0] == "eyes":
                        moves.append('e')
                        if len(parts)>=2:
                                moves.append(parts[1])
                        else:
                                print 'ERROR: you have to add a number for the comand Eyes in line '+str(lineCount)
                                return -1

                        numCmd=numCmd+2
                        cmdExpected="cmdOrEndMovement"

                elif parts[0] == "head":
                        moves.append('h')
                        try:
                                moves.append(parts[1])
                                moves.append(parts[2])
                                moves.append(parts[3])
                                moves.append(parts[4])
                        except:
                                print "ERROR: you have to add up to 4 values to the head command. Line "+str(lineCount)
                                return -1

                        numCmd = numCmd + 5
                        cmdExpected="cmdOrEndMovement"

                elif parts[0] == "head_relative":
                        moves.append('v')
                        try:
                                moves.append(parts[1])
                                moves.append(parts[2])
                                moves.append(parts[3])
                                moves.append(parts[4])
                        except:
                                print "ERROR: you have to add up to 4 values to the head command. Line "+str(lineCount)
                                return -1

                        numCmd = numCmd + 5
                        cmdExpected="cmdOrEndMovement"

                elif parts[0] == "wait":
                        moves.append('w')
                        if len(parts) >=2:
                                moves.append(parts[1])
                        else:
                                print 'ERROR: you have to add a number for the comand wait in line '+str(lineCount)
                                return -1

                        numCmd = numCmd + 2
                        cmdExpected="cmdOrEndMovement"


                elif parts[0] == "say":
                        moves.append('s')
                        if len(parts)>=2:
                                moves.append(parts[1:])
                        else:
                                print 'ERROR: you have to add a number for the comand say in line '+str(lineCount)
                                return -1

                        numCmd = numCmd + 2
                        cmdExpected="cmdOrEndMovement"                        
                        
                elif parts[0] == "mouth":
                        moves.append('o')
                        if len(parts)>=2:
                                moves.append(parts[1])
                        else:
                                print 'ERROR: you have to add a string for the comand say in line '+str(lineCount)
                                return -1

                        numCmd = numCmd + 2
                        cmdExpected="cmdOrEndMovement"

                elif parts[0] == "expression":
                        moves.append('x')

                        if len(parts)>=2:
                                moves.append("_replace_"+parts[1])
                        else:
                                print 'ERROR: you have to add a string for the comand expression in line '+str(lineCount)
                                return -1

                        numCmd = numCmd + 2
                        cmdExpected="cmdOrEndMovement"



	# Replacement phase. 
        # Inside each expression we might have a call to another expression, so we get all its movements and we copy them into its place.
	auxList = str(script)
	for key in script.keys():
		replace = "'_replace_"+key+"'" 

		text2replace = str(script[key])

		for key2 in script.keys():
			if replace in str(script[key2]):
				text = str(script[key2])
				newText = text.replace(replace, text2replace)
				script[key2] = newText
			
        return script

def feeling():       
        
        # Loading feelings from the file 
        global feelinglist 
        feelinglist = loadFeelings() 
       
        rospy.init_node('qbo_expressions')
       
        rospy.Subscriber('/Qbo/runExpression', String, runFeeling, queue_size=1)
      
        #small test and reset of mouth
        hread = MainThread(['m o none m ]'])
        hread.start()
  
	rospy.spin()

if __name__ == '__main__':
        try:
                print "qbo_expressions ON"
                feeling()
        except rospy.ROSInterruptException: pass
