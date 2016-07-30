#!/usr/bin/env python

from mock import MagicMock, patch
import sys
import unittest

import rospy
import rostest
from std_srvs.srv import *

from needybot.control.task_server.base import Task 
from needybot.control.task_server.boot_sequence import BootSequence
from needybot.control.task_server.task_server import TaskServer 
from needybot.control.task_server.task_server_client import TaskServerClient

from needybot.lib.logger import *

from needybot_srvs.srv import NextTaskRequest

class MyBootSequence(BootSequence):

    def __init__(self):
        super(MyBootSequence, self).__init__()
        self.completed = False

    def run(self):
        loginfo("Running My Boot Sequence")
        rospy.sleep(0.5)
        self.completed = True

class MyTaskServer(TaskServer):

    def __init__(self):
        super(TaskServer, self).__init__('my')
        self.add_boot_sequence(MyBootSequence)

class TestTaskServer(unittest.TestCase): 

    @patch('needybot.control.task_server.boot_sequence.BootSequence')
    def setUp(self, MockSequence):
        server = TaskServer('needybot')
        self.server_client = TaskServerClient('needybot')

        self.boot = MockSequence()
        server.add_boot_sequence(self.boot)

        self.idle = Task("idle")
        self.task_one = Task("task_one")

        self.idle.steps['load'].entered_handler = MagicMock()
        self.idle.instruct = MagicMock()

        self.task_one.steps['load'].entered_handler = MagicMock()
        self.task_one.steps['abort'].entered_handler = MagicMock()
        self.task_one.instruct = MagicMock()

        self.server_client.boot(EmptyRequest())
        rospy.sleep(0.1)
    
    def tearDown(self):
        self.server_client.shutdown(EmptyRequest())

        self.idle.shutdown()
        self.task_one.shutdown()
        rospy.sleep(0.1)

    def test_boot(self):
        self.boot.run.assert_called_once() 
        self.idle.steps['load'].entered_handler.assert_called_once() 

    def test_abort(self):

        # Since we're mocking the abort_entered, we'll need to make sure it calls abort from the mock
        def side_effect(step):
            self.idle.abort()

        self.idle.steps['abort'].entered_handler = MagicMock(side_effect=side_effect)

        rospy.sleep(0.2)
        self.server_client.abort(NextTaskRequest(next_task='task_one'))

        rospy.sleep(0.2)
        self.idle.steps['abort'].entered_handler.assert_called_once() 
        self.task_one.steps['load'].entered_handler.assert_called_once() 

    def test_complete(self):

        # Since we're mocking the completed_entered, we'll need to make sure it calls task_shutdown from the mock
        def side_effect(step):
            self.idle.task_shutdown()

        self.idle.steps['complete'].entered_handler = MagicMock(side_effect=side_effect)

        rospy.sleep(0.2)
        self.server_client.complete(NextTaskRequest(next_task='task_one'))

        rospy.sleep(0.2)
        self.idle.steps['complete'].entered_handler.assert_called_once() 
        self.task_one.steps['load'].entered_handler.assert_called_once() 


if __name__ == '__main__':
    rospy.loginfo("-I- task server test started")
    rospy.init_node('test_task_server')
    rostest.rosrun('needybot_test', 'test_task_server', TestTaskServer, sys.argv)
