#!/usr/bin/env python2.6
# -*- coding: utf-8 -*-
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
import urllib2

import sys


#transcriptionASCII=sys.argv[2].encode('ASCII')
#print transcriptionASCII
#transcription=sys.argv[2].decode("utf-8")
#print transcription
data="data=userName:"+sys.argv[1]+",transcription:"+urllib2.quote(sys.argv[2])+",lang:"+sys.argv[3]+",fromBot:true";
#data="data={\"userName\":\""+sys.argv[1]+"\", \"transcription\":\""+  urllib2.quote(sys.argv[2])   +"\", \"lang\":\""+  sys.argv[3]  +"\", \"fromBot\": true  }";
print "TRANSCRIPTION SENDER"
print data
#data=data.encode("ASCII")
#test=urllib2.quote(data)
#print test
print data
req = urllib2.Request("http://"+sys.argv[4]+":"+sys.argv[5]+"/save", data)
f = urllib2.urlopen(req)
response = f.read()

#print response
