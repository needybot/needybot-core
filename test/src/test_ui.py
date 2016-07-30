#!/usr/bin/env python

import json
from mock import MagicMock, patch
import sys
import threading
import unittest

from rosbridge_library.capabilities import subscribe
from rosgraph.masterapi import Master
import rospy
import rostest
from std_msgs.msg import String
from std_srvs.srv import *

from needybot.communication.ui import UI, UIClient

from needybot.lib import cable 
import needybot.lib.channels as nb_channels
from needybot.lib.logger import *

from needybot_srvs.srv import * 

class UINode(threading.Thread):

    def __init__(self):
        super(UINode, self).__init__()
        self.is_shutdown = False

    def run(self):
        self.ui = UI()
        while not self.is_shutdown:
            pass

    def shutdown(self):
        self.is_shutdown = True 
        self.ui.shutdown()


class TestUI(unittest.TestCase): 

    def setUp(self):
        self.client = UIClient()
        self.node = UINode()
        self.node.start()

        self.msg = None
        '''
        self.instruct_pub = rospy.Publisher(
            nb_channels.Messages.instruct.value,
            String,
            queue_size=10
        )
        '''

        self.subscribe()

        rospy.sleep(0.1)

    def handle_instruct(self, msg):
        self.msg = msg

    def subscribe(self):
        self.instruct_sub = rospy.Subscriber(
            nb_channels.Messages.instruct.value,
            String,
            self.handle_instruct
        )

    def tearDown(self):
        self.node.shutdown()

    def test_ready(self):
        rospy.sleep(0.1)
        self.assertTrue(self.node.ui.is_connected)
        self.instruct_sub.unregister()
        rospy.sleep(0.1)
        self.assertFalse(self.node.ui.is_connected)
        self.subscribe()
        rospy.sleep(0.1)
        self.assertTrue(self.node.ui.is_connected)

    def test_send(self):
        response = self.client.send(UIMessageRequest(
            topic=nb_channels.Messages.instruct.value,
            json='{"foo": "bar"}'
        ))

        self.assertTrue(response.success)

        rospy.sleep(0.1)
        self.assertIsNotNone(self.msg.data)

        json_data = json.loads(self.msg.data)
        self.assertEqual(json_data.get("foo"), "bar")

        response = self.client.send(UIMessageRequest(
            topic=nb_channels.Messages.instruct.value,
            json='{malformed: True}'
        ))
        self.assertFalse(response.success)

        # Test cable lib
        cable.instruct({
            'instructed': True
        })
        rospy.sleep(0.1)

        json_data = json.loads(self.msg.data)
        self.assertTrue(json_data.get("instructed"))

   
if __name__ == '__main__':
    rospy.loginfo("-I- ui test started")
    rospy.init_node('test_ui')
    rostest.rosrun('needybot_test', 'test_ui', TestUI, sys.argv)
