#!/usr/bin/env python

import rospy
import rostest
import sys
import unittest

class TestNeedybotBlackboard(unittest.TestCase): 

    def setUp(self):
        pass

    def tearDown(self):
        pass


if __name__ == '__main__':
    rospy.loginfo("-I- needybot blackboard test started")
    rospy.init_node('test_needybot_blackboard')
    rostest.rosrun('needybot_test', 'test_needybot_blackboard', TestNeedybotBlackboard, sys.argv)
