#!/usr/bin/env python
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2011 Thecorpora, S.L.
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
# Authors: Miguel Angel Julian <miguel.a.j@openqbo.com>

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

    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,stderr=subprocess.PIPE)

   
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

    print "PID: "+str(p.pid)

    ph_out, ph_err = p.communicate()
    
    print "Finished"

    return (ph_out, ph_err, ph_ret)
