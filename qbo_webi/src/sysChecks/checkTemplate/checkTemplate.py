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
from sysChecks.chekers.chekers import chekers
#incluir dependencias de ROS

class checkTemplate(chekers):

    def __init__(self,lang):
        #Este es obligatorio
        chekers.__init__(self,'checkTemplate',lang)

        #Codigo ROS

        #Este es obligatorio. Modificar si es necesario la direccion y nombre del template
        self.htmlTmpl = Template(filename='sysChecks/chekers/'+self.name+'/templates/checkTemplateTemplate.html')

        #Esto si no existe no se pone
        #self.jsTmpl = Template(filename='sysChecks/chekers/checkTemplate/templates/checkTemplateTemplate.js')
        #self.cssTmpl = Template(filename='sysChecks/chekers/checkTemplate/templates/checkTemplateTemplate.css')

    #redefine in is necesary
    #def get_html_content(self):
        #return self.htmlTmpl.render(language=self.language)
