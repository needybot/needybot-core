import rospy
import std_msgs.msg as ros_msg

from needybot.lib.logger import *
from needybot.lib.patterns import Singleton

from needybot_srvs.srv import *

class NeedybotBlackboard(object):
    """
    Defines a singleton for storing properties across nodes.

    Propertties that are defined are stored as rosparams so their
    values can be accessed across threads.

    :Example:

        @property
        def goal_name(self):
            return self.get_param('goal_name')

        @goal_name.setter
        def goal_name(self, val):
            self.set_param('goal_name', val)

    """

    __metaclass__ = Singleton

    def __init__(self):
        self.publishers = {}
        rospy.on_shutdown(self.shutdown)

    # Appends a value to a rosparam list
    def append_to(self, param, item):
        param_list = getattr(self, param)
        param_list.append(item)
        setattr(self, param, param_list)

    def get_param(self, param_name, default=None):
        return rospy.get_param('/needybot/blackboard/{}'.format(param_name), default)

    def publish(self, publisher_name):
        if self.publishers.get(publisher_name) == None:
            self.publishers[publisher_name] = rospy.Publisher(
                '/needybot/blackboard/{}'.format(publisher_name),
                ros_msg.Empty,
                queue_size=1
            )
            rospy.sleep(0.1)

        self.publishers[publisher_name].publish(ros_msg.Empty())

    def register_scanned_uuid(self, uuid, fullname=None):
        if self.pending_scan_type == self.PARTICIPANT:
            self.participant = uuid
            self.participant_name = fullname
        elif self.pending_scan_type == self.GOAL:
            self.goal = uuid

    def set_param(self, param_name, val):
        # Set private var value
        setattr(self, '_{}'.format(param_name), val)
        if val == None:
            full_param_name = '/needybot/blackboard/{}'.format(param_name)
            if rospy.has_param(full_param_name):
                rospy.delete_param(full_param_name)
            self.publish("delete_{}".format(param_name))
        else:
            rospy.set_param('/needybot/blackboard/{}'.format(param_name), val)
            self.publish("set_{}".format(param_name))

    def shutdown(self):
        """
        Override this method to clear out unset any properties on shutdown.
        """
        pass

    @property
    def priority_task(self):
        return self.get_param('priority_task')

    @priority_task.setter
    def priority_task(self, val):
        self.set_param('priority_task', val)

    '''
    Property the indicates if when a task is successful, if the robot
    should attempt the goal to perform another task for them, such as
    take them to a specific person or type of person.
    '''
    @property
    def sequence_tasks(self):
        return self.get_param('sequence_tasks')

    @sequence_tasks.setter
    def sequence_tasks(self, val):
        self.set_param('sequence_tasks', val)

    @property
    def simulate(self):
        return self.get_param('simulate')

    @simulate.setter
    def simulate(self, val):
        self.set_param('simulate', val)

    @property
    def step_success(self):
        return self.get_param('step_success')

    @step_success.setter
    def step_success(self, val):
        self.set_param('step_success', val)
