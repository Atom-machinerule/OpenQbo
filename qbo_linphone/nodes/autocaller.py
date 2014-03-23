#!/usr/bin/env python

import roslib; roslib.load_manifest('qbo_linphone')
import rospy

from linphone.linphone import Linphone
from qbo_linphone.srv import CallMe

import shutil

class Autocaller:
    def __init__(self):

	path = roslib.packages.get_pkg_dir("qbo_linphone")

        rospy.init_node('autocaller')
        audio_device_id = rospy.get_param('audio_device', 'ALSA: default device')
	host = rospy.get_param('linphone_host','127.0.0.1')

	#host = "192.168.1.35"

	botName = rospy.get_param('linphone_botName','default_botName_linphone')

	print "LINPHONE "+botName+"@"+host

	#reconfiguramos el fichero de configuracion /config/linphonc.in
	part2= open(path+"/config/linphonerc_part2.in",'r');

	shutil.copyfile(path+"/config/linphonerc_part1.in",path+"/config/linphonerc.in");


	configFile = open(path+"/config/linphonerc.in",'a')

	configFile.write("reg_identity=sip:"+botName+"@"+host+"\n")

	for line in part2:
		configFile.write(line)

	part2.close
	configFile.close


        self.phone = Linphone( audio_dev_id = audio_device_id,
                               termination_condition = rospy.is_shutdown,
			       _host = host,
			       _botName = botName )
        call_service = rospy.Service('autocaller', CallMe, self.handle_call_me)
        print "Ready to call."

    def handle_call_me(self, call_request):
        return self.phone.call(call_request.address)

if __name__ == '__main__':
    try:
        caller = Autocaller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

