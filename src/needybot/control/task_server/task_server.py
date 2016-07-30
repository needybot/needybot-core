#!/usr/bin/env python

import json
import random
import re
import rospy
import threading
import uuid

from geometry_msgs.msg import PoseWithCovarianceStamped 
from rospy.exceptions import ROSInterruptException
from std_msgs import msg as ros_msg
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

import pi_trees_lib.pi_trees_lib as pt 
import pi_trees_ros.pi_trees_ros as pt_ros 

from base import Task, TaskStepStatus 

from needybot.control.trees.tasks import TaskRunner, TaskSpawner 
from needybot.control.trees.blackboard import Blackboard
from needybot.services.base import BaseService

from needybot.lib.decorators import handle_shutdown_exception
import needybot.lib.channels as nb_channels
from needybot.lib import cable 
from needybot.lib.logger import * 
from needybot.lib.needybot_blackboard import NeedybotBlackboard
from needybot.lib.patterns import Singleton
from needybot.lib.utils import build_payload

from needybot_msgs.msg import UIResponse 
from needybot_srvs.srv import *

SERVICES = [
    ('abort', NextTask),
    ('boot', Empty),
    ('complete', NextTask),
    ('fail_step', Empty),
    ('shutdown', Empty),
    ('succeed_step', Empty),
]

class TaskServer(BaseService):

    def __init__(self, ns='', idle_task='idle'):
        rate = rospy.Rate(50)
        namespace = 'task_server' if len(ns) == 0 else '{}_task_server'.format(ns)
        super(TaskServer, self).__init__(namespace, rate)
        rospy.on_shutdown(self.shutdown)

        self.is_shutdown = False

        self.boot_sequences = []
        self.idle_task = idle_task
        self.task_name = idle_task 

        self.proxy_services = [
            ['abort', AbortTask],
            ['step_name', Message],
            ['next_step', NextStep],
            ['task_payload', TaskPayload],
            ['reset', Empty],
            ['step', StepTask],
            ['status', TaskStatus]
        ]

        self.register_services(SERVICES)

        self.nb_blackboard = NeedybotBlackboard()
        self.blackboard = Blackboard()
        self.blackboard.server = self
        self.task_step_serve_sub = None 

        self.task_sequence = pt.Sequence("main")

        self.task_queue_selector = pt.Selector("task_queue")
        self.task_runner = TaskRunner(self.blackboard) 
        self.task_spawner = TaskSpawner("task_spawner", self.blackboard)

        self.task_queue_selector.add_child(self.task_runner)
        self.task_queue_selector.add_child(self.task_spawner)

        self.task_sequence.add_child(self.task_queue_selector)

        self.response_sub = rospy.Subscriber(
            nb_channels.Messages.response.value,
            UIResponse,
            self.handle_usbscreen_response
        )

    def abort(self, rqst):
        self.task_name = rqst.next_task if rqst.next_task else self.idle_task
        self.blackboard.step_proxy("abort")
        return NextTaskResponse()

    def add_boot_sequence(self, sequence):
        self.boot_sequences.append(sequence)

    def assign_task_proxies(self, task_name):
        """
        Assigns service proxies for specified task.

        Args:
            task_name(str): Name of task to lookup services for. Will assign proxies by
                            looking for service function with formatted string <task_name>_<service_name>
        """
        for task_arr in self.proxy_services: 
            service_name = "{}_{}".format(task_name, task_arr[0])
            #loginfo("WAITING ON SERVIVCE {}".format(service_name))
            rospy.wait_for_service(service_name)
            #loginfo("DONE WAITING ON SERVIVCE {}".format(service_name))
            setattr(
                self.blackboard,
                "{}_proxy".format(task_arr[0]),
                rospy.ServiceProxy(
                    service_name,
                    task_arr[1]
                )
            )

        if self.task_step_serve_sub:
            self.task_step_serve_sub.unregister()

        # Subscribe to topics to failure or success of task steps
        self.task_step_serve_sub = rospy.Subscriber(
            '/needybot/{}/step/serve'.format(task_name),
            ros_msg.String,
            self.task_step_serve_handler
        )
        # Call reset on task
        self.blackboard.reset_proxy(EmptyRequest())

    def boot(self, rqst=None):
        """
        Ensure that the USB screen services are available before starting
        """

        self.is_shutdown = False

        for sequence in self.boot_sequences:
            sequence.run()

        self.nb_blackboard.priority_task = None
        self.nb_blackboard.sequence_tasks = False 

        """
        Start-up-and-spin method that creates subscribers, serves a task,
        then blocks the thread to keep it alive.
        """
        '''
        if block:
            self.run() 
        else:
            run_thread = threading.Thread(target=self.run)
            run_thread.start()
        '''
        run_thread = threading.Thread(target=self.run)
        run_thread.start()
        return EmptyResponse()

    def complete(self, rqst):
        self.task_name = rqst.next_task if rqst.next_task else self.idle_task 
        self.blackboard.step_proxy("complete")
        return NextTaskResponse()

    def fail_step(self, rqst):
        next_step_name = self.blackboard.next_step_proxy(
            status=TaskStepStatus.FAILURE,
        ).name
        try:
          self.blackboard.step_proxy(next_step_name)
        except Exception as e:
          logerr("Failed to serve step {}. {}".format(next_step_name, e))
        return EmptyResponse()
      
    def handle_usbscreen_response(self, msg):
        """
        Callback function for /needybot/msg/response topic. 

        Args:
            msg(needybot/UIResponse): stringified json object containing payload data send from usb screen 
        """
        
        if not self.blackboard.current_task_name:
            return

        step_name = msg.step
        status = msg.status
        # Log step success in case task step needs it in a callback
        self.nb_blackboard.step_success = status

        if not status in [TaskStepStatus.SUCCESS, TaskStepStatus.FAILURE]: 
            logerr("Problem retrieving status from usbscreen. Make sure it matches a constant of type TaskStepStatus.") 
            return

        # TODO Check if current step of task matches msg.step
        # if self.blackboard.next_step_proxy(MessageRequest()) == msg.step:
        # current_step=msg.step
        next_step_name = self.blackboard.next_step_proxy(status=status).name

        # If not next step specified for response type, abort the current task
        if next_step_name and len(next_step_name):
            loginfo("Handling usbscreen response in task server. Message Step: {}, Message Status: {}".format(msg.step, msg.status))
            req = StepTaskRequest(name=next_step_name)
            res = self.blackboard.step_proxy(req)
            if not res.success:
                rospy.logerr("*** Unable to step task. Aborting. ***")
                self.blackboard.abort_proxy()

    def run(self):
        while not self.is_shutdown and not rospy.is_shutdown():
            try:
                self.task_sequence.run()
                self.blackboard.rate.sleep()
            except ROSInterruptException:
                break

    def serve_task(self):

        """
        Handles selecting and serving a task (i.e., publishing to a task-
        specific topic).

        If a task is provided, it will run that task, and will otherwise
        select the next task in the task queue.

        By default, each task that is run will be cycled to the back of
        the queue for reuse later.

        If the last task sent back data that can be used in the next task,
        such as whom to address in the next task, it will be added to the
        task dictionary before publishing.

        Kwargs:
            past_data (dict): dictionary object containing any data that was
                returned from the previous task
        """

        if self.is_shutdown: 
            return None 

        loginfo("Serve task {}".format(self.task_name))

        self.assign_task_proxies(self.task_name)

        res = self.blackboard.step_proxy(name='load')

        if res.success:
            return (self.task_name, str(uuid.uuid4()))
        else:
            rospy.logerr("Failed to load task")
            return None 

    def shutdown(self, rqst=None):
        super(TaskServer, self).shutdown()

        self.blackboard.current_task_name = None
        for task_arr in self.proxy_services: 
            setattr(
                self.blackboard,
                "{}_proxy".format(task_arr[0]),
                None
            )

        self.is_shutdown = True
        self.task_name = self.idle_task 

        if self.response_sub:
            self.response_sub.unregister()
            self.response_sub = None

        if self.task_step_serve_sub:
            self.task_step_serve_sub.unregister()
            self.task_step_serve_sub = None

        return EmptyResponse()

    def succeed_step(self, rqst):
        next_step_name = self.blackboard.next_step_proxy(
            status=TaskStepStatus.SUCCESS,
        ).name
        try:
          self.blackboard.step_proxy(next_step_name)
        except Exception as e:
          logerr("Failed to serve step {}. {}".format(next_step_name, e))
        return EmptyResponse()

    def usb_screen_send(self, topic, step, context=None, did_fail=False):
        """
        Helper function to send messages via the USB Screen service

        Args:
            topic(str): topic value to send over to the usb screen
            step(str): name of the step we want to inform the usb screen about
            context(str): context of this message (almost always the task name) 
            did_fail(bool): whether or not this is a failure message 
        """

        if context == None and self.blackboard.current_task_name is not None:
            context = self.blackboard.current_task_name

        res = cable.publish(
            topic,
            step=step,
            context=context,
            did_fail=did_fail
        )

        """
        Service responds with false success if app or ipad somehow disconnected
        This could be a good place to enter an emergency sequence since ostensibly
        this means we have no screen for the robot.
        """
        if not res.success:
            rospy.logerr("Failed to send {} topic to USB screen.".format(topic.value))

    ############
    # Handlers #
    ############
    def task_step_serve_handler(self, msg):
        loginfo("Task step serve handler. Message Step: {}".format(msg.data)) 
        next_step_name = msg.data 
        try:
          self.blackboard.step_proxy(next_step_name)
        except Exception as e:
          logerr("Failed to serve step {}. {}".format(next_step_name, e))


if __name__ == '__main__':
    rospy.init_node('task_server')
    task_server = TaskServer()
    task_server.boot()
