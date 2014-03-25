#!/usr/bin/env python2.6
# coding: utf-8
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
import os
import glob
from mako.template import Template
from tabsClass import TabClass
import rospy

class SettingsManager(TabClass):
    
    def __init__(self,language):
        self.language = language
        self.htmlTemplate = Template(filename='settings/templates/settingsTemplate.html')
        self.languages_names={'en':'English','es':'Espa&ntilde;ol','pt':'Portugu&ecirc;s','de':'Deutsch','fr':'Français','it':'Italiano'}
#        self.languages_names={'en':'English','es':'Español','pt':'Português','de':'Deutsch','fr':'Français','it':'Italiano'}

    def get_languages(self):
        langlist=[]
        for fname in glob.glob("lang/*.txt"):
            langlist.append(fname[5:-4])
        langlist.sort()
        return langlist


    @cherrypy.expose
    def index(self):
        all_lang=self.get_languages()
        return self.htmlTemplate.render(language=self.language,lannames=self.languages_names,alllanguage=all_lang).encode('utf-8')


