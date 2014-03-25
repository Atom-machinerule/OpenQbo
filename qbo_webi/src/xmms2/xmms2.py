#!/usr/bin/env python2.7
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

# coding: utf-8
import cherrypy
import json
import os
import subprocess
from mako.template import Template
from tabsClass import TabClass
import roslib
roslib.load_manifest('qbo_webi');
import rospy
import time
from xmmsclient import sync, XMMSError, PLAYBACK_STATUS_PAUSE, PLAYBACK_STATUS_PLAY, PLAYBACK_STATUS_STOP
from types import *
import cgi
import tempfile


info_values = ['id','title','artist','album','url']


''' 
    aux classes to help in the upload proccess
    source: http://tools.cherrypy.org/wiki/DirectToDiskFileUpload 
'''
class myFieldStorage(cgi.FieldStorage):
    """Our version uses a named temporary file instead of the default
    non-named file; keeping it visibile (named), allows us to create a
    2nd link after the upload is done, thus avoiding the overhead of
    making a copy to the destination filename."""
    
    def make_file(self, binary=None):
        return tempfile.NamedTemporaryFile()


def noBodyProcess():
    """Sets cherrypy.request.process_request_body = False, giving
    us direct control of the file upload destination. By default
    cherrypy loads it to memory, we are directing it to disk."""
    cherrypy.request.process_request_body = False

cherrypy.tools.noBodyProcess = cherrypy.Tool('before_request_body', noBodyProcess)
# remove any limit on the request body size; cherrypy's default is 100MB
# (maybe we should just increase it ?)
cherrypy.server.max_request_body_size = 0

# increase server socket timeout to 60s; we are more tolerant of bad
# quality client-server connections (cherrypy's defult is 10s)
cherrypy.server.socket_timeout = 60



''' from here the normal stuff '''
class XMMS2Manager(TabClass):
    
    def __init__(self,language):
        subprocess.Popen(['/usr/bin/xmms2d'])
        self.language = language
        self.songsPath=roslib.packages.get_pkg_dir("qbo_webi")+"/src/xmms2/songs/"
        self.htmlTemplate = Template(filename='xmms2/templates/xmms2Template.html')
        self.jsTemplate = Template(filename='xmms2/templates/xmms2Template.js')
        self.c = sync.XMMSSync("Web")
        self.c.connect(os.getenv("XMMS_PATH"))


    def getSortSongs(self,playlist,values=info_values):
        order=self.c.playlist_list_entries(playlist)
        playlistsongs=self.getSongs(playlist,values)
        for song in playlistsongs:
           pos=order.index(song['id'])
           order.pop(pos)
           order.insert(pos,song)
        return order

    def getSongs(self,playlist,values=info_values):
        return self.c.coll_query_infos( self.c.coll_get(playlist,"Playlists"),values)


    def getPlayLists(self):
        playlists=self.c.playlist_list()
        playlists.remove("_active")
        return playlists

        
    @cherrypy.expose
    def moveSong(self,oldpos,newpos):
        self.c.playlist_move( int(oldpos), int(newpos))

    def play(self):
        self.c.playback_start()
        return self.getSelectedSong()

    @cherrypy.expose
    def getSelectedSong(self):
#        t0 = time.time()
        status=str(self.c.playback_current_id())
#        print "Selected Song "+str(time.time() - t0)+"seconds"
        return status
#str(self.c.playback_current_id())

    def pause(self):
        self.c.playback_pause()
        return self.getSelectedSong()
         
    @cherrypy.expose
    def setVolume(self,volume):
        t0 = time.time()
        if self.c.playback_status()!=0:
            self.c.playback_volume_set("Master",int(volume))
        else:
            self.play()
            self.pause()
            time.sleep(0.01)
            self.c.playback_volume_set("Master",int(volume))
            self.stop()
#        print "Set volume"+str(time.time() - t0)+"seconds"

    @cherrypy.expose
    def playSong(self,ident):
        self.pause()
        nowPlaying=self.c.playback_current_id()
        print "Play song id:"+str(ident)
        print "Now Playing:"+str(nowPlaying)
        songslist=self.getSortSongs("_active",['id','title','artist','album','url'])
        count=0
        dstPos=-1
        srcPos=-1
        for song in  songslist:
            if song['id']==int(ident):
                dstPos=count
            elif song['id']==nowPlaying:
                srcPos=count
            if dstPos!=-1 and srcPos!=-1:
                break
            count=count+1
        self.c.playlist_set_next_rel(dstPos-srcPos)
        self.c.playback_tickle()
        self.play()


    @cherrypy.expose
    def getVolume(self):
        if self.c.playback_status()!=0:
            volume=self.c.playback_volume_get()
        else:
            self.play()
            self.pause()
            time.sleep(0.01)
            volume=self.c.playback_volume_get()
            self.stop()
        return str(volume['master'])

    @cherrypy.expose
    def playpause(self):
        if self.c.playback_status()==1:
            self.pause()
        else:
            self.play()
        return str(self.c.playback_status())

    @cherrypy.expose
    def getStatus(self):
        return str(self.c.playback_status())


    @cherrypy.expose
    def stop(self):
        self.c.playback_stop()

    @cherrypy.expose
    def next(self):
        try:
            self.c.playlist_set_next_rel(1)
            self.c.playback_tickle()
	    print "Parece que todo OK en NEXT "
        except XMMSError:
	    print "ERROR en next de xmms2"
            pass
        return self.getSelectedSong()

    @cherrypy.expose
    def previous(self):
        try:
            self.c.playlist_set_next_rel(-1)
            self.c.playback_tickle()
        except XMMSError:
            pass
        return self.getSelectedSong()

    @cherrypy.expose
    def getActivePlaylistSongs(self):
        songslist=self.getSortSongs("_active",['id','title','artist','album','url'])
        for song in  songslist:
            print song['title']
            if song['title']==None:
                print song['url']
                fileFields=song['url'].split('/')
                print fileFields
                song['title']=fileFields.pop()
        return json.dumps(songslist)

    @cherrypy.expose
    def delete(self,ident):
        playlistsongs=self.getSortSongs("_active")
        counter=0
        for song in playlistsongs:
            if int(song['id'])==int(ident):
                self.c.playlist_remove_entry(counter)
            counter=counter+1
        return  self.getActivePlaylistSongs()



    @cherrypy.expose
    def xmms2Js(self, **params):
        return self.jsTemplate.render(language=self.language)

    
    @cherrypy.expose
    def index(self):
        return self.htmlTemplate.render(language=self.language)

    @cherrypy.expose
    @cherrypy.tools.noBodyProcess()
    def upload(self, theFile=None):

        print "uploading"
            
        """upload action
        
        We use our variation of cgi.FieldStorage to parse the MIME
        encoded HTML form data containing the file."""
        
        # the file transfer can take a long time; by default cherrypy
        # limits responses to 300s; we increase it to 1h
        cherrypy.response.timeout = 3600
        
        # convert the header keys to lower case
        lcHDRS = {}
        for key, val in cherrypy.request.headers.iteritems():
            lcHDRS[key.lower()] = val
        
        # at this point we could limit the upload on content-length...
        # incomingBytes = int(lcHDRS['content-length'])
        
        # create our version of cgi.FieldStorage to parse the MIME encoded
        # form data where the file is contained
        formFields = myFieldStorage(fp=cherrypy.request.rfile,
                                    headers=lcHDRS,
                                    environ={'REQUEST_METHOD':'POST'},
                                    keep_blank_values=True)
        
        # we now create a 2nd link to the file, using the submitted
        # filename; if we renamed, there would be a failure because
        # the NamedTemporaryFile, used by our version of cgi.FieldStorage,
        # explicitly deletes the original filename
        theFile = formFields['theFile']



        #detect whether the input is only one file or more
        list = False
        try:
            for f in theFile:
                break
            list = True
        except:
            list = False


        if list:
            for f in theFile:
                try:
                    os.link(f.file.name, self.songsPath+f.filename)
                except:
                    ''' file already exists, so we do nothing '''
        else:
            try:
                os.link(theFile.file.name, self.songsPath+theFile.filename)
            except:
                ''' file already exists, so we do nothing '''


#        self.c.medialib_add_entry(self.songsPath+theFile.filename)
#        print self.c.medialib_get_id(self.songsPath+theFile.filename)
        print type(theFile)
        if type(theFile) is ListType:
            for songFile in theFile:
                self.c.medialib_add_entry("file://"+self.songsPath+songFile.filename)
                self.c.playlist_add_id(self.c.medialib_get_id("file://"+self.songsPath+songFile.filename))
        else:
            self.c.medialib_add_entry("file://"+self.songsPath+theFile.filename)
            self.c.playlist_add_id(self.c.medialib_get_id("file://"+self.songsPath+theFile.filename)) 
        
#        self.c.playlist_add_url("file://"+self.songsPath+theFile.filename,"_active")
        return '<form id="Form" action="/" method="post"><input type="hidden" name="activeTab" value="5" /></form> <script>document.getElementById("Form").submit();</script>'

     

