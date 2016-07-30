import json
import re
import rospy

from rosapi.srv import Subscribers 

from needybot.communication.ui import UIClient 
import needybot.lib.channels as nb_channels 
from needybot.lib.logger import *

from needybot_srvs.srv import *

def publish(topic, instructions=None):
    if type(topic) not in [str, nb_channels.Messages]:
        rospy.logerr("** Topic must be of type string or Messages in order to publish to cable **")
        return False

    if instructions and type(instructions) is not dict:
        rospy.logerr("** Instructions must be of type dictionary in order to publish to cable **")
        return False

    # Properly format topic string
    topic_val = topic if type(topic) is str else topic.value 

    try:
        rospy.wait_for_service('ui_send', 0.1)

        client = UIClient()
        return client.send(
            topic_val,
            json.dumps(instructions) 
        )
    except rospy.ROSException:
        return UIMessageResponse(success=False) 

def instruct(instructions):
    res = publish(
        nb_channels.Messages.instruct.value,
        instructions=instructions
    )

    """
    Service responds with false success if app or ipad somehow disconnected
    This could be a good place to enter an emergency sequence since ostensibly
    this means we have no screen for the robot.
    """
    if not res.success:
        rospy.logerr("*** Failed to send {} topic to USB screen. ***".format(nb_channels.Messages.instruct.value))


