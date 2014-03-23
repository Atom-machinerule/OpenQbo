#!/usr/bin/env python
import subprocess
import time
import re

class LinphoneError(Exception):
    def __init__(self, message):
        self.message = message
    def __str__(self):
        return self.message

# Run a shell command and return stdout and stderr together as a string.
# Python 2.7 has subprocess.check_output(), but I am using only 2.6.5.
def check_output(args, **kwds):
  kwds.setdefault("stdout", subprocess.PIPE)
  kwds.setdefault("stderr", subprocess.STDOUT)
  p = subprocess.Popen(args, **kwds)
  return p.communicate()[0].strip()

class Linphone():
    # Constructor.  Starts Linphone daemon if one is not already
    # running and waits for it to become ready.  If
    # termination_condition is given, it is a function which returns
    # True when (otherwise) infinite loops should end.
    def __init__(self,
                 audio_dev_id = "ALSA: default device",
                 termination_condition = lambda: False,
		 _host = '192.168.1.35',
		 _botName = "default_botName"):

        self.termination_condition = termination_condition

        configuration = open("/opt/ros/electric/stacks/qbo_stack/qbo_linphone/config/linphonerc.in").read()
        configuration = configuration.replace('%AUDIO_DEV_ID%', audio_dev_id)
        config_file = open("/tmp/linphonerc", 'w')
        config_file.write(configuration)
        config_file.close()

        ready = False
        while not ready and not self.termination_condition():
            # Start linphone daemon
            init_output = check_output(["linphonecsh", "init",
                                        "-c", "/tmp/linphonerc",
                                        "-l", "/tmp/linphone.log",
                                        "-d", "1"])
            print "init_output =", init_output
            if init_output != "" and not re.search("A running linphonec has been found", init_output):
                print "init failed."
                raise LinphoneError(init_output)

            print "init seems to have worked."
            # Wait for it to become ready
            count = 0
            exit_code = subprocess.call(["linphonecsh", "status", "hook"])
            while( exit_code == 255
                   and not self.termination_condition()
                   and count < 20 ):
                time.sleep(0.1)
                count += 1
                exit_code = subprocess.call(["linphonecsh", "status", "hook"])
            if exit_code != 255:
                check_output(["linphonecsh", "register", "--host",_host,"--username",_botName,"--password","qbobot"])
                self.autoanswer()
                ready = True

    # Destructor.  Attempts to shutdown Linphone daemon.
    def __del__(self):
        try:
            print "linphone exit"
            subprocess.call(["linphonecsh", "exit"]);
        except:
            pass

    # Ask Linphone to hangup and then make a call to an address and
    # wait until the call is either established or fails.  Returns
    # True on success, False on failure, or throws a LinphoneException
    # if the hook status is not understood.
    def call(self, address):
        subprocess.call(["linphonecsh", "hangup"])

        print "calling ", address
        subprocess.call(["linphonecsh", "generic", "call " + address])
        while not self.termination_condition():
            time.sleep(1)
            hook_status = check_output(["linphonecsh", "status", "hook"])
            print "calling ", address, "hook status is", hook_status
            if hook_status == "hook=offhook":
                return False
            elif re.search("^Call out", hook_status):
                return True
            elif hook_status == "hook=dialing":
                pass
            else:
                raise LinphoneError("Unknown hook status: " + hook_status)

    # Enable autoanswer.
    def autoanswer(self):
        result = check_output(["linphonecsh", "generic", "autoanswer enable"])
        print "autoanswer result =", result

    def in_a_call(self):
        hook_status = check_output(["linphonecsh", "status", "hook"])
        if hook_status == "hook=offhook":
            return False
        elif re.search("^Call out", hook_status):
            return True
        elif re.search("^hook=answered", hook_status):
            return True
        else:
            raise LinphoneError("Unknown hook status: " + hook_status)
