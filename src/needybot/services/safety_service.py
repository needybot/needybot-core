#!/usr/bin/env python

import json

import rospy
import std_msgs.msg as ros_msg

from kobuki_msgs.msg import BumperEvent, CliffEvent, SensorState, WheelDropEvent

from needybot.core.services.base import BaseService, BaseServiceClient
from needybot.services.event_service import EventClient, EventTypes
import needybot.lib.channels as nb_channels
from needybot.lib import slack
from needybot.lib.logger import * 

from needybot_srvs.srv import SafetyService, SafetyServiceResponse
from needybot_msgs.msg import Safety


NAMESPACE = 'safety'
SERVICES = [
    ('start', SafetyService),
    ('stop', SafetyService),
    ('safety_check', SafetyService)
]


class SafetyClient(BaseServiceClient):

    """
    Class for interacting directly with the Safety Service that informs
    other nodes of Needy's safety at reqeust time.
    """

    def __init__(self):
        super(SafetyClient, self).__init__(NAMESPACE, SERVICES)

    
class SafetyService(BaseService):

    """
    Service class responsible for reporting Needy's safety level.
    """

    def __init__(self, namespace, rate):
        super(SafetyService, self).__init__(namespace, rate)
        self.register_services(SERVICES)
        
        self.bumper_event = BumperEvent()
        self.cliff_event = CliffEvent()
        self.wheel_drop_event = WheelDropEvent()
        self.low_battery_alerted = False
        
        rospy.Subscriber(
            '/mobile_base/events/bumper',
            BumperEvent,
            self.bumper_callback
        )
        rospy.Subscriber(
            '/mobile_base/events/cliff',
            CliffEvent,
            self.cliff_detected_callback
        )
        rospy.Subscriber(
            '/mobile_base/events/wheel_drop',
            WheelDropEvent,
            self.wheel_drop_callback
        )
        rospy.Subscriber(
            '/mobile_base/sensors/core',
            SensorState,
            self.power_callback
        )

        self.bumper_states = [BumperEvent.RELEASED, BumperEvent.RELEASED, BumperEvent.RELEASED]
        self.cliff_states = [CliffEvent.FLOOR, CliffEvent.FLOOR, CliffEvent.FLOOR]
        self.wheel_states = [WheelDropEvent.RAISED, WheelDropEvent.RAISED]
        self.kobuki_base_max_charge = rospy.get_param('/needybot/kobuki_base_max_charge', 160)

        self.ground_pub = rospy.Publisher(
            nb_channels.Safety.ground.value,
            Safety,
            queue_size=10
        )

        self.obstacle_pub = rospy.Publisher(
            nb_channels.Safety.obstacle.value,
            Safety,
            queue_size=10
        )

        self.picked_up_pub = rospy.Publisher(
            nb_channels.Safety.picked_up.value,
            ros_msg.Empty,
            queue_size=10
        )

        self.put_down_pub = rospy.Publisher(
            nb_channels.Safety.put_down.value,
            ros_msg.Empty,
            queue_size=10
        )

        self.picked_up = False 
        self.safe_on_ground = True
        self.safe_from_obstacles = True
        
    def bumper_callback(self, data):

        """
        Subscriber handler for '/mobile_base/events/bumper'.

        Assigns own properties based on the BumperState message received.

        Theses values are assessed in self.perform, where appropriate action
        is taken under certain circumstances.
        """

        self.bumper_event = data
        if data.state == 1:
            EventClient.capture_event(EventTypes.BUMPER, True)
        else:
            pass

    def cliff_detected_callback(self, data):
        
        """
        Subscriber handler for '/mobile_base/events/cliff'.

        Assigns own properties based on the CliffState message received.
        
        These values are assessed in self.perform, where appropriate action
        is taken under certain circumstances.
        """

        self.cliff_states[data.sensor] = data.state
        if data.state == CliffEvent.CLIFF:
            # EventClient.capture_event(EventTypes.CLIFF, True)
            self.ground_unsafe()
        elif self.safe_on_ground == False:
            self.check_ground_safe()

    def check_ground_safe(self):
        # Ignore if flag already set
        if self.safe_on_ground == True: return

        for state in self.cliff_states:
            if state == CliffEvent.CLIFF:
                return

        for state in self.wheel_states:
            if state == WheelDropEvent.DROPPED:
                return

        self.safe_on_ground = True
        self.ground_pub.publish(Safety(safe=True))

    def check_picked_up(self):
        if self.picked_up:
            for val in self.wheel_states:
                if val == WheelDropEvent.DROPPED: return
            self.picked_up = False
            self.put_down_pub.publish()
            EventClient.capture_event(EventTypes.PUT_DOWN)
        else:
            for val in self.wheel_states:
                if val == WheelDropEvent.RAISED: return
            self.picked_up = True 
            self.picked_up_pub.publish()
            EventClient.capture_event(EventTypes.PICKED_UP)

    def ground_unsafe(self):
        # Ignore if flag already set
        if self.safe_on_ground == False: return

        self.safe_on_ground = False
        self.ground_pub.publish(Safety(safe=False))

    def power_callback(self, data):
        percent_left = round(float(data.battery) / float(self.kobuki_base_max_charge) * 100)
        if self.low_battery_alerted == False and percent_left <= 8 and int(data.charger) == 0:
            self.low_battery_alerted = True
            slack.sos("Help me friends! Needy's battery is low. Needy is slowing down. Needy's legs hurt. This is not good!")
            return
        elif self.low_battery_alerted == True and int(data.charger) != 0:
            self.low_battery_alerted = False 
            slack.sos("Awwweee yeah! Thank you loyal human friends. Needy is safe now...getting juiced!")
            return

        if percent_left > 15:
            self.low_battery_alerted = False 

    def safety_conditions(self):

        """
        Checks each sensor of note and returns a prioritized opinion of what
        the least safe issue is, ranging from cliff, to bumper, to wheel (drop).

        Returns:
             SafetyServiceResponse: the response containing the robot's safety
                 report for the movement action being performed.
        """

        res = SafetyServiceResponse()
        res.safe = False
        res.bumper_event = BumperEvent()
        res.cliff_event = CliffEvent()
        res.wheel_event = WheelDropEvent()
        
        if self.cliff_event.state:
            res.cliff_event = self.cliff_event
        elif self.bumper_event.state:
            res.bumper_event = self.bumper_event
        elif self.wheel_drop_event.state:
            res.wheel_event = self.wheel_drop_event
        else:
            res.safe = True
        return res
            
    def safety_check(self, req):

        """
        Service method that returns an int value describing the safety
        conditions Needybot is encountering.

        Returns [0, None] if safe, returns one of several response codes if unsafe
        conditions are met. Reaction to this message can be all-or-nothing,
        i.e., the robot is safe or isn't, or a granular reaction, where
        the unsafe_result value is considered and action is taken
        based on the conditions of the danger.

        Args:
            req (needybot.srv.SafetyServiceRequest): the request sent to
                the service.

        Returns:
            SafetyServiceResponse: the state of the safety of Needybot
        """

        return self.safety_conditions()

    def start(self, msg):
        raise NotImplementedError('Method undefined for the safety service.')

    def stop(self, msg):
        raise NotImplementedError('Method undefined for the safety service.')

    def wheel_drop_callback(self, data):

        """
        Subscriber handler for '/mobile_base/events/wheel_drop'.
        """

        self.wheel_states[data.wheel] = data.state
        if data.state == WheelDropEvent.DROPPED:
            self.ground_unsafe()
        elif self.safe_on_ground == False:
            self.check_ground_safe()

        self.check_picked_up()

            
if __name__ == '__main__':
    rospy.init_node('safety_service')

    rate = rospy.Rate(50)
    safety_service = SafetyService(NAMESPACE, rate)
    while not rospy.is_shutdown():
        safety_service.rate.sleep()
