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
# Authors: Miguel Angel Julian <miguel.julian@openqbo.com>;
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
#          Arturo Bajuelos <arturo@openqbo.com>; 
#          Sergio Merino <s.merino@openqbo.com>;


import cherrypy
from mako.template import Template
from tabsClass import TabClass
import json
import os
import glob

def my_import(name):
    mod = __import__(name)
    components = name.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

class sysChecksManager(TabClass):
    def __init__(self,lang):
        self.lang=lang['current_language']#lang
        self.htmlTemplate = Template(filename='sysChecks/templates/sysChecksTemplate.html')

        #creo la lista de chekers
        self.availableChekers={}
        checkersDirs=next(os.walk('sysChecks/chekers'))[1]
        if 'templates' in checkersDirs:
            checkersDirs.remove('templates')
        for i in range(len(checkersDirs)):
            checkersDirs[i]='sysChecks/chekers/'+checkersDirs[i]
        filesToImport=[]
        for direc in checkersDirs:
            fileToImport=glob.glob(direc+'/[a-zA-Z]*.py')
            module = my_import(fileToImport[0][:-3].replace('/','.'))
            meth = fileToImport[0][:-3].split('/')[-1]
            code=getattr(module,meth)
            self.availableChekers[meth]=code(self.lang)

        #Actualizo los lenguajes
        self.set_language(lang)
#        self.language={}
#        for checkerKey in self.availableChekers.keys():
#          self.language.update(self.availableChekers[checkerKey].language)


    def set_language(self,lang):
        self.lang=lang['current_language']
        for checker in self.availableChekers.values():
            checker.load_language(self.lang)
        self.language={}
        for checkerKey in self.availableChekers.keys():
          self.language.update(self.availableChekers[checkerKey].language)


    @cherrypy.expose
    def index(self, *cheker):
        print cheker,' len: ',len(cheker)
        if not cheker:
            return self.htmlTemplate.render(language=self.language, availableChekers=self.availableChekers)
        if cheker[0] in self.availableChekers.keys():
            returnDataDic={}
            returnDataDic['htmlElement']=self.availableChekers[cheker[0]].get_html(cheker[1:])
            js=self.availableChekers[cheker[0]].get_js()
            css=self.availableChekers[cheker[0]].get_css()
            if js:
                returnDataDic['jsElement']=js
            if css:
                returnDataDic['cssElement']=css
            return json.dumps(returnDataDic)
        if cheker[0]=='undefined':
            returnDataDic={}
            returnDataDic['htmlElement']=''
            return json.dumps(returnDataDic)
        else:
            print 'No check available',self.availableChekers.keys()
            return json.dumps(False)

