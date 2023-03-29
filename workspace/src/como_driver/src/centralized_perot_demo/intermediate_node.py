#!/usr/bin/env python

import roslib
import rospy
from barc.msg import ECU
from como_driver.srv import Choice,ChoiceResponse

class ECUPublish:
	def __init__(self):
		self.ecu = ECU(0.,0.)
		self.ecu_pub = rospy.Publisher('/ecu', ECU, queue_size = 1)
		
	def set_ecu(self, motor, servo):
		self.ecu = ECU(float(motor), float(servo))
	
	def publish_ecu(self):
		self.ecu_pub.publish(self.ecu)

class ECUSubscribe:
	def __init__(self):
		self.ecu = ECU(0.,0.)
		self.str_sub = rospy.Subscriber("/temp_ecu", ECU, self.callback, queue_size =1)
		
	def callback(self, data):
		self.ecu = ECU(data.motor, data.servo)
		
	def get_ecu(self):
		return self.ecu

class ChoiceSVR:
    def __init__(self):
        self.choice = True
        rospy.Service('choice_response', Choice, self.respond_to_choice)

    def respond_to_choice(self, req):
        print("Returning [%s ]" % (req.state))
        if req.state == "start":
            self.choice = True
            return ChoiceResponse(True)
        else:
            self.choice = False
            return ChoiceResponse(False)

    def get_choice(self):
        return self.choice


def main():
	rospy.init_node("intermediate_node", anonymous = True)
	ecuPublish = ECUPublish()
	ecuSubscribe = ECUSubscribe()
	rate = rospy.Rate(30)
	choiceServer = ChoiceSVR()

	while not rospy.is_shutdown():
		ecu_transfer = ecuSubscribe.get_ecu()
		choice_result = choiceServer.get_choice()

		if choice_result == False:
			ecuPublish.set_ecu(0, ecu_transfer.servo)
		else:
			ecuPublish.set_ecu(ecu_transfer.motor, ecu_transfer.servo)

		ecuPublish.publish_ecu()

		rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down line_follower_ctrl node")
		pass

