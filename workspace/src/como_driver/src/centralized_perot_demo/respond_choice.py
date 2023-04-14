#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from como_driver.srv import *



def respondChoice(x, y):
    rospy.wait_for_service('choice_response')
    try:
        respond_to_choice = rospy.ServiceProxy('choice_response', Choice)
        resp1 = respond_to_choice(x, y)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "usage"


if __name__ == "__main__":

    if len(sys.argv) == 3:
        x = (sys.argv[1])
        y = (sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("[Requesting...]  state: %s | namespace: %s" % (x, y))
    print("%s" % (respondChoice(x, y)))

