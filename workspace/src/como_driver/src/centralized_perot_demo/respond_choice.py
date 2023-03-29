#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from como_driver.srv import *


def respondChoice(x):
    rospy.wait_for_service('choice_response')
    try:
        respond_to_choice = rospy.ServiceProxy('choice_response', Choice)
        resp1 = respond_to_choice(x)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "usage"


if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = (sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s" % (x))
    print("State is %s | Result is %s" % (x, respondChoice(x)))

