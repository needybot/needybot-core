#!/usr/bin/env python

from mock import MagicMock, patch
import sys
import unittest

import rospy
import rostest
from std_srvs.srv import *

from needybot.control.task_server.base import Task, TaskStep, TaskStepStatus
from needybot.control.task_server.boot_sequence import BootSequence
from needybot.control.task_server.task_server import TaskServer 
from needybot.control.task_server.task_server_client import TaskServerClient

from needybot.lib import cable 
from needybot.lib.logger import *

from needybot_srvs.srv import * 

class MyTask(Task):

    def __init__(self):
        super(MyTask, self).__init__('mytask')
        self.add_steps([
            TaskStep(
                'load',
                success_step='say_hello',
                entered_handler=self.load_entered,
                instructions={
                    'view': 'eye',
                    'type': 'happy'
                }
            ),
            TaskStep(
                'say_hello',
                failure_step='load',
                success_step='say_goodbye',
                entered_handler=self.hello_entered,
                exited_handler=self.hello_exited
            ),
            TaskStep(
                'say_goodbye',
                failure_step='abort',
                success_step='complete',
                entered_handler=self.goodbye_entered,
                exited_handler=self.goodbye_exited
            ),
            TaskStep(
                'timed',
                failure_step='abort',
                success_step='complete',
                timeout_step='say_hello',
                timeout=0.1,
                entered_handler=self.timed_entered,
                exited_handler=self.timed_exited
            )

        ])

    def goodbye_entered(self, step):
        pass

    def goodbye_exited(self, step, next_step):
        pass

    def hello_entered(self, step):
        pass

    def hello_exited(self, step, next_step):
        pass

    def timed_entered(self, step):
        pass

    def timed_exited(self, step, next_step):
        pass


class TestTask(unittest.TestCase): 

    def setUp(self):
        self.my_task = MyTask()
        self.server = TaskServer(idle_task='mytask')
        cable.instruct = MagicMock()

        rospy.sleep(0.1)

        self.client = TaskServerClient()
        self.client.boot(EmptyRequest())

        rospy.sleep(0.1)
    
    def tearDown(self):
        self.client.shutdown(EmptyRequest())
        self.my_task.shutdown()
        rospy.sleep(0.1)

    def test_failure_step(self):
        hello_entered = MagicMock()
        hello_exited = MagicMock()
        load_entered = MagicMock()

        self.my_task.steps['load'].entered_handler = load_entered 
        self.my_task.steps['say_hello'].entered_handler = hello_entered
        self.my_task.steps['say_hello'].exited_handler = hello_exited 

        self.client.succeed_step(EmptyRequest())
        self.assertEqual(self.my_task.current_step.name, 'say_hello')

        self.client.fail_step(EmptyRequest())
        hello_entered.assert_called_once()
        hello_exited.assert_called_once()
        load_entered.assert_called_once()

    def test_instruct(self):
        cable.instruct.assert_called_once()

    def test_services(self):

        '''
        Test varius services that aren'ts tested individually
        '''

        rospy.wait_for_service("mytask_step_name")
        proxy = rospy.ServiceProxy(
            "mytask_step_name",
            Message 
        )

        response = proxy(MessageRequest())
        self.assertEqual(response.value, 'load')

        proxy = rospy.ServiceProxy(
            "mytask_task_payload",
            TaskPayload 
        )

        response = proxy()
        self.assertEqual(response.status, TaskStatusRequest.RUNNING)
        self.assertFalse(response.did_fail)

        proxy = rospy.ServiceProxy(
            "mytask_next_step",
            NextStep 
        )

        response = proxy(status=TaskStepStatus.SUCCESS)
        self.assertEqual(response.name, 'say_hello')
        response = proxy(status=TaskStepStatus.FAILURE)
        self.assertEqual(response.name, 'abort')


    def test_status(self):
        rospy.wait_for_service("mytask_status")
        proxy = rospy.ServiceProxy(
            "mytask_status",
            TaskStatus 
        )

        response = proxy(TaskStatusRequest())
        self.assertEqual(response.status, TaskStatusRequest.RUNNING)

    def test_success_step(self):
        entered = MagicMock()
        exited = MagicMock()

        self.my_task.steps['say_hello'].entered_handler = entered
        self.my_task.steps['say_hello'].exited_handler = exited 

        self.client.succeed_step(EmptyRequest())
        self.assertEqual(self.my_task.current_step.name, 'say_hello')

        entered.assert_called_once()
        self.client.succeed_step(EmptyRequest())
        exited.assert_called_once()

    def test_timeout(self):
        entered = MagicMock()
        exited = MagicMock()

        self.my_task.steps['timed'].entered_handler = entered
        self.my_task.steps['timed'].exited_handler = exited 

        rospy.wait_for_service("mytask_step")
        proxy = rospy.ServiceProxy(
            "mytask_step",
            StepTask 
        )

        response = proxy(name='timed')
        self.assertEqual(self.my_task.current_step.name, 'timed')
        entered.assert_called_once()

        rospy.sleep(0.2)
        self.assertEqual(self.my_task.current_step.name, 'say_hello')
        exited.assert_called_once()

        # Test that the timer does in fact reset
        rospy.sleep(0.2)
        self.assertEqual(self.my_task.current_step.name, 'say_hello')


if __name__ == '__main__':
    rospy.loginfo("-I- task test started")
    rospy.init_node('test_task')
    rostest.rosrun('needybot_test', 'test_task', TestTask, sys.argv)
