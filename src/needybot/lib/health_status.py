#!/usr/bin/env pythonm

import json

import kobuki_msgs.msg as kobuki_msg
import rospy
import smart_battery_msgs.msg as battery_msg
import std_msgs.msg as ros_msg

from needybot_msgs.msg import Battery


class HealthStatus(object):

    """
    Class that keeps track of health status of Needybot ecosystem.

    May in the future be:
        * absorbed into another class
        * report other information than just battery status
    """

    def __init__(self):
        self.battery_threshold = rospy.get_param('/mobile_base/battery_dangerous')
        self.battery_level = None
        self.ipad_battery_threshold = rospy.get_param('/needybot/ipad/battery_threshold', 0.2)
        self.ipad_battery = None

        self.kobuki_sub = rospy.Subscriber('/mobile_base/sensors/core',
                                           kobuki_msg.SensorState,
                                           self.kobuki_battery_level)
        self.ipad_sub = rospy.Subscriber('needybot/status/ipad_battery',
                                         Battery,
                                         self.ipad_battery_level)

    def kobuki_battery_level(self, data):

        """
        Subscriber handler for '/mobile_base/sensors/core' that records the
        Kobuki base's current battery level.

        Args:
            data (kobuki_msgs.msg.SensorState): data payload from ROS topic
        """

        self.kobuki_battery_level = data.battery_level
        
    def ipad_battery_level(self, msg):

        """
        Subscriber handler for '/needybot/status/ipad_battery' that records the
        iPad's current battery level.

        Args:
            msg (needybot/Battery): message from iPad containing battery level
        """

        self.ipad_battery = msg.battery

    def status(self):

        """
        Prints the status of the three batteries to stdout.

        ** In future, may print other health conditions.**
        """

        rospy.loginfo('Kobuki base battery level: {}'.format(self.battery_level))
        rospy.loginfo('iPad battery level: {}'.format(self.ipad_battery))
        

if __name__ == '__main__':
    rospy.init_node('health_status')
    health_status = HealthStatus()
    health_status.status()
