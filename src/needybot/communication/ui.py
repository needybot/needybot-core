#!/usr/bin/env python

import json
import re
import threading

from rosgraph.masterapi import Master
import rospy
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import Empty, String

from needybot.services.base import BaseServiceClient, BaseService

import needybot.lib.channels as nb_channels
from needybot.lib.logger import *

from needybot_srvs.srv import UIMessage 
from rosapi.srv import Subscribers 

NAMESPACE = '/ui'
SERVICES = [
    ('send', UIMessage)
]


class UIClient(BaseServiceClient):

    def __init__(self):
        super(UIClient, self).__init__(NAMESPACE, SERVICES)


class UI(BaseService):

    def __init__(self):
        super(UI, self).__init__(NAMESPACE)
        self.did_timeout = False
        self.is_connected = False
        self.keep_alive_timer = None
        self.register_services(SERVICES)

        self.publishers = {}

        self.connected_pub = rospy.Publisher(
            nb_channels.Messages.ui_connected.value,
            Empty,
            queue_size=10,
            latch=True
        )

        self.disconnected_pub = rospy.Publisher(
            nb_channels.Messages.ui_disconnected.value,
            Empty,
            queue_size=10,
            latch=True
        )


        self.connection_monitor = threading.Thread(target=self.monitor)
        self.connection_monitor.start()

    def monitor(self):
        # Wait until subscriber on instruct message is present
        notified = False
        while not rospy.is_shutdown():
            _, subscribers, _ = Master('/needybot').getSystemState()
            if dict(subscribers).get(nb_channels.Messages.instruct.value) is not None:
                if self.is_connected == False:
                    self.connected_pub.publish()
                self.is_connected = True
            else:
                if self.is_connected or not notified:
                    notified = True
                    self.disconnected_pub.publish()
                self.is_connected = False 

            rospy.sleep(0.1)

    def send(self, req):
        # TODO May want to wait for iPad to connect here
        if not self.is_connected:
            logwarn('iPad application is not yet connected.')
            return False

        if req.json:
            try:
                valid_json = json.loads(req.json)
            except:
                logerr('Invalid JSON sent to UI service.')
                return False

        self.send_topic(req.topic, req.json)
        return True

    def send_topic(self, topic, json_data):
        pub = self.publishers.get(topic, None) 
        if not pub:
            pub = rospy.Publisher(
                topic,
                String,
                queue_size=10,
                latch=True
            )
            self.publishers[topic] = pub
            rospy.sleep(0.1)

        pub.publish(json_data)


if __name__ == '__main__':
    try:
        rospy.init_node('ui', anonymous=False)
        UI()
        rospy.spin()
    except ROSInterruptException:
        rospy.loginfo('UI is shutting down...')
