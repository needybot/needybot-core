import rospy
import threading

from actionlib_msgs.msg import GoalStatus
from enum import Enum

class ActionGoalState(Enum):

    """
    Simple Enum class that gives us a way to access the names of
    the state returned by the `SimpleActionClient.get_state` method.

    That method returns one of eight statuses, which are used as values
    in this enum.
    """

    pending = GoalStatus.PENDING
    active = GoalStatus.ACTIVE
    preempted = GoalStatus.PREEMPTED
    succeeded = GoalStatus.SUCCEEDED
    aborted = GoalStatus.ABORTED
    rejected = GoalStatus.REJECTED
    recalled = GoalStatus.RECALLED
    lost = GoalStatus.LOST


class ActionGoalThread(threading.Thread):

    """
    Class responsible for managing the lifecycle of an ActionGoal
    via threading. When instantiated, it takes a service to run,
    a duration to allow the service to complete, and a callback
    that is executed once either the service has completed or
    the duration has expired.

    Init Args:
        service (actionlib.SimpleActionClient): the service that will be run
            and acts as the goal for this threaded process
        duration (rospy.Duration): the duration inside of which the service
            is allowed to return a response to the goal thread.
        callback (function): the callback function to trigger once either
            the service has returned or the duration has expired.
    """
    
    def __init__(self, service, duration, callback):
        self.done = False
        self.duration = rospy.Duration(duration)
        self.running = True
        self.service = service
        self.state = None
        self.callback = callback
        super(ActionGoalThread, self).__init__()

    def run(self):

        """
        The main thread loop for the ActionGoalThread object. At the end of the
        super().__init__() method is where self.run() is called.

        Blocks the thread until either a result is returned from the service
        or the duration has expired.

        Then proceeds to record the service state, and execute its callback.
        """
        
        # this will block until a result is acquired from the ActionServer
        self.finished_in_time = self.service.wait_for_result(self.duration)

        if self.running:
            self.done = True
            # add one to the service state value, since we are using
            # a one-based index
            self.state = ActionGoalState(self.service.get_state())
            self.callback()

    def stop(self):

        """
        Toggles the self.running property to off, to prevent the main block
        in the object's run method from executing. Also cancels the service
        if it has not completed yet.
        """
        
        self.running = False
        if not self.done:
            self.service.cancel_goal()
