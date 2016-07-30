#!/usr/bin/env python

import json
import re

import rospy
from std_msgs.msg import String
from ws4py.client.threadedclient import WebSocketClient


class NeedyClient(WebSocketClient):

    joined = False

    def __init__(self, url):
        WebSocketClient.__init__(self, url)
        rospy.init_node('socket_client', anonymous=False)
        rospy.on_shutdown(self.shutdown)

    def opened(self):
        rospy.loginfo('[Socket]: Opened')
        if not self.join():
            rospy.loginfo('[Socket]: Already joined')
        self.push({ 'mood': 'happy' })

    def closed(self, code, reason=None):
        rospy.loginfo('[Socket]: Closed')

    def received_message(self, message):
        rospy.loginfo('message: {}'.format(str(message)))

        if re.search('needy:follow', str(message)):
            rospy.loginfo('follow message received')
            follow_publisher = rospy.Publisher('camera/follow_state',
                                               String, queue_size=10)
            follow_publisher.publish('toggle')

    def join(self):
        if self.joined:
            return False
        self.joined = True
        data = {
            'topic': 'needy:init',
            'event': 'phx_join',
            'ref': '',
            'payload': {}
        }
        self.send(json.dumps(data))
        return True

    def push(self, message):
        data = {
            'topic': 'needy:init',
            'event': 'needy:status',
            'ref': '',
            'payload': message
        }
        self.send(json.dumps(data))

    def shutdown(self):
        self.close()

if __name__ == '__main__':

    try:
        ws = NeedyClient('ws://nb-services.herokuapp.com/ws')
        ws.connect()
        ws.run_forever()
        rospy.loginfo('Socket client connected')
    except rospy.ROSInterruptException:
        ws._shutdown()
        rospy.loginfo('Socket client finished')
