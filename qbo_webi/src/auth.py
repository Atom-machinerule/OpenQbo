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
# Authors: Miguel Angel Julian <miguel.julian@openqbo.com>;
#          Daniel Cuadrado <daniel.cuadrado@openqbo.com>;
#          Arturo Bajuelos <arturo@openqbo.com>; 
#          Sergio Merino <s.merino@openqbo.com>;

# -*- encoding: UTF-8 -*-
#
# Form based authentication for CherryPy. Requires the
# Session tool to be loaded.
#

import cherrypy
import roslib; roslib.load_manifest('qbo_webi')
import PAM
import subprocess

def runCmd(cmd, timeout=None):
    '''
    Will execute a command, read the output and return it back.
    
    @param cmd: command to execute
    @param timeout: process timeout in seconds
    @return: a tuple of three: first stdout, then stderr, then exit code
    @raise OSError: on missing command or if a timeout was reached
    '''

    ph_out = None # process output
    ph_err = None # stderr
    ph_ret = None # return code

    p = subprocess.Popen(cmd, shell=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    # if timeout is not set wait for process to complete
    if not timeout:
        ph_ret = p.wait()
    else:
        fin_time = time.time() + timeout
        while p.poll() == None and fin_time > time.time():
            time.sleep(1)

        # if timeout reached, raise an exception
        if fin_time < time.time():

            # starting 2.6 subprocess has a kill() method which is preferable
            # p.kill()
            os.kill(p.pid, signal.SIGKILL)
            raise OSError("Process timeout has been reached")

        ph_ret = p.returncode


    ph_out, ph_err = p.communicate()

    return (ph_out, ph_err, ph_ret)

SESSION_KEY = '_cp_username'

def verify_password(user, password):
    print 'cyphered credentials:'
    print '  user: '+user
    print '  pass: '+password

    user = user.strip()
    password = password.strip()

    passpath = roslib.packages.get_pkg_dir("qbo_webi")+'/src/robotpass'
    f = open(passpath, "r")
    robotpass = f.read().strip()

    (username, err, ret)=runCmd("echo "+user+" | openssl enc -aes-256-cbc -pass pass:"+robotpass+" -d -base64")
    (passw, err, ret)=runCmd("echo "+password+" | openssl enc -aes-256-cbc -pass pass:"+robotpass+" -d -base64")

    print 'decyphered user credentials:'
    print '  user: '+username
    print '  pass: '+passw

    def pam_conv(auth, query_list, userData):
        resp = []
        resp.append( (passw, 0))
        return resp
    res = False
    service = 'passwd'

    auth = PAM.pam()
    auth.start(service)
    auth.set_item(PAM.PAM_USER, username)
    auth.set_item(PAM.PAM_CONV, pam_conv)
    try:
        auth.authenticate()
        auth.acct_mgmt()
    except PAM.error, resp:
        print 'Go away! (%s)' % resp
        res = False 
    except:
        print 'Internal error'
        res = False
    else:
        print 'Good to go!'
        res = True

    return res

def check_credentials(username, password):
    """Verifies credentials for username and password.
    Returns None on success or a string describing the error on failure"""
    # Adapt to your needs
    #path = roslib.packages.get_pkg_dir("qbo_webi")
    #f = open(path+"/config/users_pwd","r")
    #try:
        ##miramos cada linea del fichero, buscando que la primera palabra coincida con el nombre
        ##y cuando asi sea, que coincida la password.
        #for line in f:
            #line = line.replace("\n","")
            #parts = line.split(" ")
            #if parts[0]==username:
                #if parts[1] == password:
                    #print 'Pass OK'
                    #return None
                #else:
                    #print 'Pass FAIL'
                    #return u"ERROR: Incorrect username or password."
    #except:
        #print 'Pass FAIL'
        #return u"ERROR: Incorrect username or password."
    #return u"ERROR: Server error."
    if verify_password(username,password):
       return None
    else:
       return u"ERROR: Incorrect username or password."
    
    # An example implementation which uses an ORM could be:
    # u = User.get(username)
    # if u is None:
    #     return u"Username %s is unknown to me." % username
    # if u.password != md5.new(password).hexdigest():
    #     return u"Incorrect password"
def check_auth(*args, **kwargs):
    """A tool that looks in config for 'auth.require'. If found and it
    is not None, a login is required and the entry is evaluated as a list of
    conditions that the user must fulfill"""
    conditions = cherrypy.request.config.get('auth.require', None)
    if conditions is not None:
        username = cherrypy.session.get(SESSION_KEY)
        if username:
            cherrypy.request.login = username
            for condition in conditions:
                # A condition is just a callable that returns true or false
                if not condition():
                    raise cherrypy.HTTPRedirect("/auth/login")
        else:
            raise cherrypy.HTTPRedirect("/auth/login")

cherrypy.tools.auth = cherrypy.Tool('before_handler', check_auth)

def require(*conditions):
    """A decorator that appends conditions to the auth.require config
    variable."""
    def decorate(f):
        if not hasattr(f, '_cp_config'):
            f._cp_config = dict()
        if 'auth.require' not in f._cp_config:
            f._cp_config['auth.require'] = []
        f._cp_config['auth.require'].extend(conditions)
        return f
    return decorate


# Conditions are callables that return True
# if the user fulfills the conditions they define, False otherwise
#
# They can access the current username as cherrypy.request.login
#
# Define those at will however suits the application.

def member_of(groupname):
    def check():
        # replace with actual check if <username> is in <groupname>
        return cherrypy.request.login == 'joe' and groupname == 'admin'
    return check

def name_is(reqd_username):
    return lambda: reqd_username == cherrypy.request.login

# These might be handy

def any_of(*conditions):
    """Returns True if any of the conditions match"""
    def check():
        for c in conditions:
            if c():
                return True
        return False
    return check

# By default all conditions are required, but this might still be
# needed if you want to use it inside of an any_of(...) condition
def all_of(*conditions):
    """Returns True if all of the conditions match"""
    def check():
        for c in conditions:
            if not c():
                return False
        return True
    return check


# Controller to provide login and logout actions

class AuthController(object):
    
    def on_login(self, username):
        """Called on successful login"""
    
    def on_logout(self, username):
        """Called on logout"""
    
    def get_loginform(self, username, msg="Enter login information", from_page="/"):
        return """<html>
        <head>
            <script src="/js/jquery-1.7.2.min.js" type="text/javascript"></script>
            <script src="/js/aes.js" type="text/javascript"></script>
            <!-- <script src="http://crypto-js.googlecode.com/svn/tags/3.0.2/build/rollups/aes.js"></script> -->

            <script>
                jQuery(document).ready(function() {
                    jQuery("#go").submit(function() {
                      var userEncoded = CryptoJS.AES.encrypt(jQuery("#username").val(), jQuery("#robotpass").val());
                      var passEncoded = CryptoJS.AES.encrypt(jQuery("#password").val(), jQuery("#robotpass").val());
                      jQuery("#username").attr("hidden","true");
                      jQuery("#password").attr("hidden","true");
                      jQuery("#username").val(userEncoded);
                      jQuery("#password").val(passEncoded);
                      jQuery("#robotpass").attr("disabled","disabled");
                      return true;
                    });
                });
            </script>
        </head>
         <body>
            <form id="go" action="/auth/login">
            <div>
                Robot username:  <input type="username" name="username" id="username"/><br />
                Robot pass:  <input type="password" name="password" id="password"/><br />
                Webi pass: <input type="password" name="robotpass" id="robotpass"/><br />
                <input type="submit" value="Log in" />
            </div><br />
            </form>
         </body>
        </html>""" % locals()
    
    @cherrypy.expose
    def login(self, username=None, password=None, from_page="/"):
        if username is None or password is None:
            return self.get_loginform("", from_page=from_page)
        
        error_msg = check_credentials(username, password)
        if error_msg:
            return self.get_loginform(username, error_msg, from_page)
        else:
            #cherrypy.session.regenerate()
            cherrypy.session[SESSION_KEY] = cherrypy.request.login = username
            self.on_login(username)
            raise cherrypy.HTTPRedirect(from_page or "/")

    @cherrypy.expose
    def mobile_login(self, username=None, password=None, from_page="/"):
        if username is None or password is None:
            return "error"

        error_msg = check_credentials(username, password)
        if error_msg:
            return "error"
        else:
            #cherrypy.session.regenerate()
            cherrypy.session[SESSION_KEY] = cherrypy.request.login = username
            self.on_login(username)
	    return "ok"

    @cherrypy.expose
    def logout(self, from_page="/"):
        sess = cherrypy.session
        username = sess.get(SESSION_KEY, None)
        sess[SESSION_KEY] = None
        if username:
            cherrypy.request.login = None
            self.on_logout(username)
        raise cherrypy.HTTPRedirect(from_page or "/")
