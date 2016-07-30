import math
import rospy
import tf

from nav_msgs.msg import Odometry

from needybot.lib.logger import *
from needybot.lib.needybot_blackboard import NeedybotBlackboard

IDLE = 0
ACTIVE = 1
INACTIVE = 2

class MovementTracker(object):

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tmr = None
        self.active_cb = None
        self.inactive_cb = None
        self.tracking = False
        self.min_dist = 0.02
        self.min_theta = 0.05
        self.state = IDLE
        self.nb_blackboard = NeedybotBlackboard()
        self.odom = None
        self.odom_sub = None
        self.pose = None
        self.prev_pose = None
        self.stopped = False

    def check_activity(self, evt):
        if self.stopped:
            return

        if not self.prev_pose:
            self.prev_pose = self.pose
            return

        dx = self.pose.position.x - self.prev_pose.position.x 
        dy = self.pose.position.y - self.prev_pose.position.y 
        delta_theta = self.pose.orientation.z - self.prev_pose.orientation.z
        dist = math.sqrt(dx*dx + dy*dy)

        if (dist < self.min_dist and delta_theta < self.min_theta) and self.state != INACTIVE:
            self.state = INACTIVE 
            self.inactive_cb()

        elif (dist >= self.min_dist or delta_theta >= self.min_theta) and self.state != ACTIVE:
            self.state = ACTIVE 
            self.active_cb() 

        self.prev_pose = self.pose

    def start(self, active_cb, inactive_cb):
        self.active_cb = active_cb
        self.inactive_cb = inactive_cb

        if self.tracking:
            logwarn("Movement tracker is already running") 
            return

        self.tracking = True

        self.odom_sub = rospy.Subscriber(
            "/odom",
            Odometry,
            self.odom_cb
        )
        self.stopped = False

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        if not self.tmr:
            self.tmr = rospy.Timer(
                rospy.Duration(0.1),
                self.check_activity
            )

    def stop(self):
        self.stopped = True 
        if self.odom_sub:
            self.odom_sub.unregister()

        if self.tmr and self.tmr.is_alive():
            self.tmr.shutdown()

        self.odom_sub = None
        self.tmr = None
        self.odom = None
        self.active_cb = None
        self.inactive_cb = None
        self.tracking = False
        self.state = IDLE
        self.pose = None
        self.prev_pose = None
