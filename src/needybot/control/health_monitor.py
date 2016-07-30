#!/usr/bin/env python

import json

import kobuki_msgs.msg as kobuki_msg
import rospy

import pi_trees_lib.pi_trees_lib as pt
import smart_battery_msgs.msg as battery_msg
import std_msgs.msg as ros_msg

from needybot.services.base import BaseService 
from needybot.lib import channels as nb_channels
from needybot.lib.patterns import Singleton

from needybot_msgs.msg import Battery
from needybot_srvs.srv import CheckHealthy, CheckHealthyRequest 

NAMESPACE = 'health_monitor'
SERVICES = [
    ('check_healthy', CheckHealthy),
]       

class HealthMonitor(BaseService):

    __metaclass__ = Singleton

    def __init__(self):
        rate = rospy.Rate(50)
        super(HealthMonitor, self).__init__(NAMESPACE, rate)
        self.ipad_healthy = True
        self.kobuki_healthy = True

        self.register_services(SERVICES)

        self.battery_threshold = rospy.get_param('/needybot/kobuki/battery_threshold', 13.2)
        self.ipad_battery_threshold = rospy.get_param('/needybot/ipad/battery_threshold', 0.2)

        self.discharging = False
        self.emergency = False

        self.emergency_sub = rospy.Subscriber(
            nb_channels.Messages.emergency.value,
            ros_msg.String,
            self.emergency_handler 
        )

        self.kobuki_sub = rospy.Subscriber('/mobile_base/sensors/core',
                                           kobuki_msg.SensorState,
                                           self.check_kobuki_battery)
        self.ipad_sub = rospy.Subscriber('needybot/status/ipad_battery',
                                         Battery,
                                         self.check_ipad_battery)

    def check_healthy(self, req):

        """
        Service function that returns whether or not the robot is healthy.

        Args:
            req(CheckHealthyRequest): Check healthy request service object. Contains the paramters of custom ROS
                                      CheckHealth.srv service type 

        Returns:
            healthy(bool): Whether or not the robot is healthy.
        """

        if self.emergency == True or self.ipad_healthy == False or self.kobuki_healthy == False:
            return False

        return True

    def check_ipad_battery(self, msg):

        """
        Subscriber handler that publishes an emergency message if the
        iPad battery is low on pwer.

        Args:
            data (needybot/Battery): the current status of the iPad battery
        """

        battery = msg.battery
        if battery and battery < self.ipad_battery_threshold:
            self.ipad_healthy = False
        else:
            self.ipad_healthy = False
        
    def check_kobuki_battery(self, data):

        """
        Subscriber handler that publishes an emergency message if the
        Kobuki base's batter is low on power.
        
        Args:
            data (kobuki_msgs.msg.SensorState): the current state of
                the Kobuki base.
        """
        
        if data.charger == kobuki_msg.SensorState.DISCHARGING:
            self.discharging = True

        if data.battery and data.battery < self.battery_threshold:
            self.kobuki_healthy = False 
        else:
            self.kobuki_healthy = True

    def emergency_handler(self, msg):

        """
        Topic handler for /needybot/msg/emergency topic

        Sets emergency flag to true which will cause cause any health status queries to return false.

        Args:
            msg(ros_msgs.String): Does nothing.
        """

        self.emergency = True

    def start(self):
        
        """
        Startup this node.
        """

        while not rospy.is_shutdown():
            self.rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('health_monitor')
    health_monitor = HealthMonitor()
    health_monitor.start()
