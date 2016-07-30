#!/usr/bin/env python

import json
import threading

import actionlib
import dynamic_reconfigure.client
from geometry_msgs.msg import PoseWithCovarianceStamped 
import rospy
import std_msgs.msg as ros_msg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse 

from kobuki_msgs.msg import WheelDropEvent

from needybot.services.base import BaseService, BaseServiceClient

from needybot.lib import cable
import needybot.lib.channels as nb_channels
from needybot.lib.logger import * 
from needybot.lib.patterns import Singleton
from needybot.lib.needybot_blackboard import NeedybotBlackboard 
from needybot.lib.task_latcher import TaskLatcher 
from needybot.lib.utils import build_payload

from needybot_srvs.srv import *

class TaskStepStatus(object):
    FAILURE = 'failure'
    RUNNING = 'running'
    SUCCESS = 'success'
    

class TaskStep(object):

    def __init__(self, name, blocking=False, timeout=0, success_step='complete', failure_step='abort',
                 timeout_step='abort', entered_handler=None, exited_handler=None, screen_delay=0, instructions=None):
            

        self.timeout = timeout 
        self.name = name                            # This step's name 
        self.success_step = success_step            # Instructions for the task server, tells it which step to move to when we recieve a success message from the usb screen 
        self.failure_step = failure_step            # Step to move to when we recieve a failure response from the usb screen 
        self.timeout_step = timeout_step            # Step to move to when we recieve a timeout response from the usb screen 
        self.entered_handler = entered_handler      # Callback for when the step is enetered 
        self.exited_handler = exited_handler        # Callback for when the step is exited 
        self.blocking = blocking
        self.instructions = instructions

        self._task_name = ''

    @property
    def task_name(self):
        return self._task_name

    @task_name.setter
    def task_name(self, val):
        self._task_name = val
        self.failure_pub = rospy.Publisher(
            '/needybot/{}/step/failure'.format(self._task_name),
            ros_msg.Empty,
            queue_size=1
        )
        self.success_pub = rospy.Publisher(
            '/needybot/{}/step/success'.format(self._task_name),
            ros_msg.Empty,
            queue_size=1
        )

    def enter(self):
        if self.entered_handler:
            self.entered_handler(self)

    def exit(self):
        if self.exited_handler:
            self.exited_handler(self)

    def fail(self):
        self.failure_pub.publish(ros_msg.Empty())

    def payload(self):
        return {
            'name': self.name,
            'success_step': self.success_step,
            'failure_step': self.failure_step,
            'timeout': self.timeout,
            'screen_delay': self.screen_delay
        }

    def succeed(self):
        self.success_pub.publish(ros_msg.Empty())


class TaskStepSequence(object):

    def __init__(self, steps, task_name, step_prefix=None):
        if len(steps) == 0:
            logerr("TaskStepSequence must have 1 or more steps.")

        self.nb_blackboard = NeedybotBlackboard()

        # Setup steps dictionary
        self.steps = {}
        self.step_prefix = step_prefix
        for step in steps:
            # Add prefix to step names
            step.name = self.prefix_name(step.name)
            self.steps[step.name] = step


        self.task_name = task_name

    def prefix_name(self, step_name):
        if self.step_prefix:
            return '{}_{}'.format(self.step_prefix, step_name) 
        return step_name

    @property
    def task_name(self):
        return self._task_name

    @task_name.setter
    def task_name(self, val):
        self._task_name = val
        self.step_serve_pub = rospy.Publisher(
            '/needybot/{}/step/serve'.format(self._task_name),
            ros_msg.String,
            queue_size=1
        )


class Task(BaseService):

    """
    Parent class for all Needybot tasks.

    Uses the singleton pattern to ensure that child tasks are only ever created
    once each.
    """

    def __init__(self, name):
        super(Task, self).__init__(name, rospy.Rate(50))

        self.nb_blackboard = NeedybotBlackboard()
        self.lock = threading.Lock()
        
        # Flags
        self.active = False
        self.completed = False
        self.did_fail = False

        self.current_step = None

        self.name = name
        self.subscribers = []
        self.step_timer = None
        self.steps = {} 

        self.task_latcher = TaskLatcher()
        self.task_latcher.register_task()

        self.step_load_time = None 
        self.paused_time = None 
        self.paused = False

        self.register_services([
            ('abort', AbortTask),
            ('step_name', Message),
            ('next_step', NextStep),
            ('task_payload', TaskPayload),
            ('reset', Empty),
            ('status', TaskStatus),
            ('step', StepTask)
        ])

        self.add_steps([
            TaskStep(
                'load',
                failure_step='abort',
                success_step='complete',
                entered_handler=self.load_entered
            ),
            TaskStep(
                'complete',
                entered_handler=self.complete_entered
            ),
            TaskStep(
                'abort',
                entered_handler=self.abort_entered,
                blocking=True
            ),
        ])

    def abort(self, req=None):
        """
        Service function for the aborting the task '[name]_abort'.

        handles a shutdown of the task but instead of calling signal_complete,
        this method calls `signal_aborted`, which lets the task server know
        that it can proceed with launching a mayday task (as opposed to queueing
        up another general task).

        Args:
            msg (std_msgs.msg.String): the message received through the
                subscriber channel.
        """
        if not self.active:
            logwarn("Can't abort {} because task isn't active.".format(self.name))
            return False

        self.instruct()
        self.prep_shutdown(did_fail=True)
        return True

    def add_step(self, instance):
        # If the array item is a squence, flatten it's steps into the steps array
        if isinstance(instance, TaskStep):
            self.steps[instance.name] = instance 
            self.steps[instance.name].task_name = self.name
            return

        for key, step in instance.steps.iteritems():
            self.steps[step.name] = step 
            self.steps[step.name].task_name = self.name

    def add_steps(self, steps):
        for step in steps:
            self.add_step(step)

    def add_subscribers(self):
        self.subscribers = [
            rospy.Subscriber(
                nb_channels.Messages.replay.value,
                ros_msg.Empty,
                self.replay_handler
            ),
            rospy.Subscriber(
                nb_channels.Messages.cancel_timer.value,
                ros_msg.Empty,
                self.cancel_timer_handler
            ),
            rospy.Subscriber(
                nb_channels.Messages.reset_timer.value,
                ros_msg.Empty,
                self.reset_timer_handler
            ),
            rospy.Subscriber(
                nb_channels.Safety.picked_up.value,
                ros_msg.Empty,
                self.picked_up_handler
            ),
            rospy.Subscriber(
                nb_channels.Safety.put_down.value,
                ros_msg.Empty,
                self.put_down_handler
            )
        ]

    def audio_done_instruct_cb(self, state, result):
        self.instruct()

    def abort_entered(self, step):
        """
        Step entered handler that aborts the task.
        """
        self.abort()

    def cancel_timer(self):
        if self.step_timer and self.step_timer.is_alive():
            self.step_timer.shutdown()

    def cancel_timer_handler(self, msg):
        self.cancel_timer()

    def complete_entered(self, step):
        """
        Step entered handler that signals the task server that the task is complete.
        """
        self.task_shutdown()
        
    def load_entered(self, step):
        """
        Step entered handler for the load step:

        where {task} is a child of this super class.

        *The incoming message is expected to arrive from the task server,
        although this class should never be invoked; this is a method for
        subclasses of this super class, referenced by {task} above.

        Responsible for loading up the task, setting initial property values,
        and beginning task/step countdown timers.

        The final steps are to publish to the iPad (and any future listeners)
        to signal that the task is ready, and then to play audio that accompanies
        the task load-up.

        Args:
            req (LoadTaskRequest): LoadTask service request 

        Returns:
            succes (Bool): Whether or not task loading was successful
        """

        self.add_subscribers()
        self.instruct()

    def pause(self):
        self.paused_time = rospy.Time.now()
        self.paused = True

        if self.step_timer:
            self.step_timer.shutdown()
            self.step_timer = None

    def picked_up_handler(self, msg):
        self.cancel_timer()

    def put_down_handler(self, msg):
        self.reset_timer()
 
    def replay_handler(self, msg):
        """
        Resets the step timer everytime the replay button is tapped on the iPad
        """
        self.update_step_timer(self.current_step)

    def reset(self, req=None):
        # Reset flags
        self.active = True
        self.did_fail = False
        self.completed = False
        
        return EmptyResponse()

    def reset_timer(self):
        self.update_step_timer(self.current_step)

    def reset_timer_handler(self, msg):
        self.reset_timer()

    def resume(self):
        if not self.paused:
          return

        self.paused = False 

        if self.current_step.timeout > 0:
          self.step_timer = rospy.Timer(
              rospy.Duration(self.current_step.timeout - (self.paused_time - self.step_load_time)),
              self.step_to_handler,
              oneshot=True
          )

        self.paused_time = None 

    def shutdown(self):
        super(Task, self).shutdown()
        self.task_latcher.unregister()

    def step_load(self, step):
        # If listeneing for any responses from the iPad, disable the subscriber
        if self.current_step and self.current_step.exited_handler:
            self.current_step.exited_handler(self.current_step, step)
        
        self.current_step = step

        self.step_load_time = rospy.Time.now()
        self.update_step_timer(self.current_step)

        self.task_latcher.step_task(self.current_step.name)

        # Checks if step should block, by default it will toss it into it's own thread
        if self.current_step.entered_handler:
            if self.current_step.blocking:
                self.current_step.entered_handler(self.current_step)
            else:
                entered_thread = threading.Thread(target=self.current_step.entered_handler, args=[self.current_step])
                entered_thread.start()

    def prep_shutdown(self, did_fail=False):

        """
        Handles:

        * resetting instance values to their defaults
        * shutting down the task and step timers
        * stopping the fishing client
        * updating the emotional value of Needybot
        * informing the iPad that the task is finished

        Kwargs:
            did_fail (bool): whether or not the task failed [Default: False]
        """

        self.active = False
        self.remove_subscribers()

        if self.step_timer and self.step_timer.is_alive():
            self.step_timer.shutdown()

        self.step_timer = None
        self.did_fail = did_fail

    def remove_subscribers(self):
        for sub in self.subscribers:
            sub.unregister()

    def step_name(self, req):
        return MessageResponse(value=self.current_step.name)

    def next_step(self, req):

        """
        Service method that takes the returned status from the iPad and returns
        either the name of the next step, or None if no step is defined for
        the current step's failure outcome

        Args:
            success_Step (bool): whether the previous step in the task was
                successful or not.

        Returns:
            string: the name of the next step, if any
            None: if no step is available, returns None
        """

        with self.lock:
            if not self.current_step:
                logwarn("Can't retrieve next step name. Current step is not set.") 
                return NextStepResponse(name='')

            step = self.current_step

            if req.status == TaskStepStatus.SUCCESS:
                return NextStepResponse(name=step.success_step)
            elif req.status == TaskStepStatus.FAILURE:
                return NextStepResponse(name=step.failure_step)
            elif req.status == TaskStepStatus.TIMEOUT:
                return NextStepResponse(name=step.timeout_step)

    def serve_step(self, name):
        step = self.steps.get(name, None)
        if not step:
            logerr("Can't find step {} of task {} to serve".format(name, self.name))
            return False

        self.step_load(step)
        return True

    def startup(self):

        """
        Helper method to fire up the task and keep it alive.
        """

        while not rospy.is_shutdown():
            self.rate.sleep()

    def status(self, req):
        """
        Service function. Returns status of this task.
        """

        with self.lock:
            return self.which_status() 
    
    def step(self, req):
        with self.lock:
            step = self.steps.get(req.name, None)
            if not step:
                logerr("Problem loading step {}. Make sure request's name is set properly.".format(req.name))
                return StepTaskResponse()

            res = StepTaskResponse()
            res.success = False

            if step:
                self.step_load(step)
                res.success = True 

            return res

    def step_to_handler(self, msg):
        if self.current_step.timeout_step:
           self.serve_step(self.current_step.timeout_step) 
        else:
          self.task_failed()

    def task_failed(self):

        """
        Method that takes care of neecessary details before
        """

        rospy.logwarn("Task failed. Shutting it down.")
        self.active = False
        self.task_shutdown(True)

    def task_payload(self, req):

        """
        Service call that returns the payload information for this task.
        Mainly used to pass data b/t running tasks.
        """

        with self.lock:
            res = TaskPayloadResponse(
                name=self.name,
                status=self.which_status(),
                did_fail=self.did_fail
            )
            return res

    def task_shutdown(self, did_fail=False):

        """
        Helper method responsible for shutting a task down.

        Resets values to their defaults, shuts down timeout counters,
        and notifies the iPad and task server node of its completion.

        Args:
            complete_handler (function): the handler method to run after
                shutting down. [Default: self.signal_complete]
        """

        self.prep_shutdown(did_fail)
        self.completed = True if not did_fail else False

    def update_step_timer(self, step):

        """
        Helper method that shuts the current step timer down, then recreates
        the step timer for the next step in the task.

        If the step timeout is zero, do not create a new timeout; otherwise
        create a new timer using the timeout value.

        Args:
            step (dict): the dictionary object representing the task step.
        """

        if self.step_timer and self.step_timer.is_alive():
            self.step_timer.shutdown()

        if not self.active:
            return

        if step and step.timeout > 0:
            self.step_timer = rospy.Timer(
                rospy.Duration(step.timeout),
                self.step_to_handler,
                oneshot=True
            )

    def which_status(self):
        status = TaskStatusRequest.INACTIVE 
        if self.did_fail:
            status = TaskStatusRequest.FAILED
        elif self.completed:
            status = TaskStatusRequest.COMPLETED 
        elif self.active:
            status = TaskStatusRequest.RUNNING
        return status


    def instruct(self):
        """
        Helper function to send messages to the iPad.

        This needs to be called manually and will be called automatically
        for every step.
        """
        if self.current_step and self.current_step.instructions:
            cable.instruct(self.current_step.instructions)
