#!/usr/bin/env python

import os
import pwd
import socket
import errno

import roslib; roslib.load_manifest('qbo_linphone')
import rospy

from linphone.linphone import Linphone
from qbo_linphone.srv import CallMe

# Request a call from an autocaller node to an address, and keep
# retrying until the service request is successful.  Returns once the
# Linphone instance on the far side thinks the call has been
# established.
def request_call(address):
    call_requested = False
    while not call_requested and not rospy.is_shutdown():
        print "Waiting for autocaller service."
        rospy.wait_for_service('autocaller')

        ask_for_call = rospy.ServiceProxy('autocaller', CallMe)
        try:
            print "Asking for a call to", address
            call_requested = ask_for_call(address)
        except rospy.ServiceException, e:
            print "Service did not process request:", str(e)
            call_requested = False

        if not call_requested:
            # Don't try again immediately in case this would become a fast loop.
            rospy.sleep(1)

# Run the call_requester node.  Keeps retrying when calls fail or are
# cut off until a ROSInterruptException, then returns.
def call_requester_node():
    username = pwd.getpwuid(os.getuid()).pw_name
    fqdn = socket.getfqdn()
    my_sip_address = "sip:" + username + "@" + fqdn
    rospy.init_node('call_requester')
    audio_device_id = rospy.get_param('audio_device', 'ALSA: default device')
    phone = Linphone( audio_dev_id = audio_device_id,
                      termination_condition = rospy.is_shutdown )
    phone.autoanswer()
    try:
        while not rospy.is_shutdown():
            request_call( my_sip_address )
            if phone.in_a_call():
                print "Call started."
                rospy.sleep(.1)
            while phone.in_a_call() and not rospy.is_shutdown():
                rospy.sleep(.1)
            print "Call ended."

    except rospy.ROSInterruptException:
        print "Interrupt!"
    except IOError as ioerr:
        if ioerr.errno == errno.EINTR:
            print "Interrupt!"
        else:
            raise

if __name__ == '__main__':
    call_requester_node()
