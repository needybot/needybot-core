import rospy

from needybot.lib.logger import *
from needybot.lib.needybot_blackboard import NeedybotBlackboard
from needybot.lib.patterns import Singleton

"""
Module that stores global vars for our behaviors trees
"""

class Blackboard(object):

    __metaclass__ = Singleton

    def __init__(self):
        self.server = None
        self.rate = rospy.Rate(50)

        self.reset()
        self.nb_blackboard = NeedybotBlackboard()

    def reset(self):
        self.in_mayday = False 
        self.current_task_name = None
        self.abort_proxy = None
        self.reset_proxy = None
        self.step_name_proxy = None
        self.next_step_proxy = None
        self.status_proxy = None
        self.step_proxy = None
        self.task_payload_proxy = None

    @property
    def current_task_id(self):
        return rospy.get_param('/needybot/current_task_id')

    @current_task_id.setter
    def current_task_id(self, val):
        rospy.set_param('/needybot/current_task_id', val)
