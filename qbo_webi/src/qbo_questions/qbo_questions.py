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
# Authors: Miguel Julian <miguel.julian@openqbo.com>; 
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
#          Sergio Merino <s.merino@openqbo.com>;
#          Arturo Bajuelos <arturo@openqbo.com>;       

import cherrypy
from mako.template import Template
from tabsClass import TabClass
import roslib; roslib.load_manifest('qbo_webi')
import rospy
import json
from qbo_talk.srv import Text2Speach

from mako.lookup import TemplateLookup


class Qbo_questionsManager(TabClass):
    def __init__(self,language):
        self.dialogue = {}
        self.dialogue_path = roslib.packages.get_pkg_dir("qbo_questions")
        self.language = language

        self.templatelookup = TemplateLookup(directories=['./'])
        self.htmlTemplate = Template(filename='qbo_questions/templates/qbo_questions.html',lookup=self.templatelookup)
        self.jsTemplate = Template(filename='qbo_questions/templates/qbo_questions.js')
        self.client_speak = rospy.ServiceProxy("/qbo_talk/festival_say", Text2Speach)


    @cherrypy.expose
    def unload(self):
        return "ok"
        
    @cherrypy.expose
    def index(self):
        #Set Festival language
        self.lang = self.language["current_language"]
        
        return self.htmlTemplate.render(language=self.language)	

    @cherrypy.expose
    def qbo_questionsJs(self,  **params):
        return self.jsTemplate.render(language=self.language)



    @cherrypy.expose
    def deleteSentence(self, lang, sentence2delete):

        print "borrando ."+sentence2delete+"."


        aux = sentence2delete.split(":::")
        check1 = aux[0]
        check2 = aux[1]

        f = open(self.dialogue_path+'/config/dialogues_'+lang)

        finalFileContent = ""
        alreayDeleted = False
        for line in f.readlines():


            aux_line = line.strip()            
            aux_line = aux_line.split(">>>")
            question =aux_line[0]
            answer =aux_line[1]

            #print question.encode('utf8').upper() +"=="+ check1.upper() +"and"+ answer.encode('utf8') +"=="+ check2.upper() +"and"+ str(not alreayDeleted)

            if check2 != "":
                #we are deleting an answer
                if question.upper() == check1.upper() and answer.upper() == check2.upper() and not alreayDeleted  :
                    alreayDeleted = True
                else:
                    finalFileContent = finalFileContent+line
            else:
                #we are deleting a question
                if question.upper() != check1.upper() :
                    finalFileContent = finalFileContent+line

        
        f.close()


        f = open(self.dialogue_path+'/config/dialogues_'+lang,'w')
        f.write(finalFileContent)
        f.close()
 
        return self.getActualDialogue(lang)

    @cherrypy.expose
    def addSentence(self, lang, question, answer):
        f = open(self.dialogue_path+'/config/dialogues_'+lang,"a") 
        f.write(question.encode('utf8') +">>>"+answer.encode('utf8')+"\n")
        f.close()

        # we check wheter the input line alreayd exists, if so, we add to its own list
        if question in self.dialogue:
           self.dialogue[question].append(answer.upper())
           self.dialogue[question].sort()
        else:
            #self.dialogue_input does not exist
            self.dialogue[question.upper()] = [answer.upper()]

        return self.getActualDialogue(lang)

    @cherrypy.expose
    def getActualDialogue(self,lang):
        self.dialogue = {}
        f = open(self.dialogue_path+'/config/dialogues_'+lang)
        for line in f.readlines():
            try:
                line=line.strip()
                if line!="":
                    parts = line.split(">>>")
                    dialogue_input = parts[0].upper()
                    dialogue_output = parts[1].upper()

                # we check wheter the input line alreayd exists, if so, we add to its own list
                    if dialogue_input in self.dialogue:
                        self.dialogue[dialogue_input].append(dialogue_output)
                        self.dialogue[dialogue_input].sort()
                    else:
                    #self.dialogue_input does not exist
                        self.dialogue[dialogue_input] = [dialogue_output]
            except ValueError:
                print "Error when creating dialog at qbo_questions tab "
                pass

        print str(self.dialogue)
        return json.dumps(self.dialogue)


    @cherrypy.expose
    def playSentence(self, answer):
        answer = answer.encode('utf8')
        print "Message to speak: "+str(answer)
        self.client_speak(str(answer))
        return "true"    

