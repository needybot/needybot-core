import datetime
import enum
import json

from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import tf


def broadcast_position(pose, to_frame, from_frame):
    broadcaster = tf.TransformBroadcaster()
    pose = pose
    broadcaster.sendTransform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        rospy.Time.now(),
        to_frame,
        from_frame
    )

def build_payload(task=None, step=None, did_fail=False, payload=None):

    """
    Builds a json payload given a named task and optional step.
    This helps standardize our messaging format both on ROS and
    on the iPad since both can expect the same payload structure.

    Args:
        task (str): the name of the current task that is
            asking to send the payload
        step (str): the step of the current task that is
            asking to send the payload
    Kwargs:
        did_fail (bool): whether or not the task failed before sending
            the payload across

    Returns:
        string: a stringified dictionary to be sent as a message payload
    """

    if not task:
        raise ValueError('[build_payload]: task must  be not NoneType')

    failure_status = did_fail

    msg_payload = {
        'task': task,
        'step': step,
        'task_failed': failure_status
    }
    if payload:
        msg_payload.update(payload)
    return json.dumps(msg_payload)

def resolve_topic_name(topic):

    """
    Given a topic of type `enum.Enum`, or of type `str`,
    get the string value of its publisher channel.

    Args:
        topic (enum.Enum | str): the topic to resolve, as
            either an enum.Enum type or a string.

    Returns:
        string: the publisher channel for the topic
        None: if the topic arg is not one of the two acceptable
            types, then None is returned
    """

    if isinstance(topic, enum.Enum):
        return topic.value
    elif isinstance(topic, str):
        return topic
    else:
        return None

'''
Translates yaml stored pose to ROS PoseWithCovarianceStamped
'''
def create_pose(pose_obj):
    pose_stamped = PoseWithCovarianceStamped()
    pose_stamped.header.frame_id = 'map'
    pose_stamped.header.stamp = rospy.Time.now() 
    
    pose = pose_obj.get("pose")
    position = pose.get("position")
    orientation = pose.get("orientation")
    covariance = pose_obj.get("covariance")

    pose_stamped.pose.pose.position.x = position.get("x") 
    pose_stamped.pose.pose.position.y = position.get("y") 
    pose_stamped.pose.pose.position.z = position.get("z") 

    pose_stamped.pose.pose.orientation.y = orientation.get("x") 
    pose_stamped.pose.pose.orientation.x = orientation.get("y") 
    pose_stamped.pose.pose.orientation.z = orientation.get("z") 
    pose_stamped.pose.pose.orientation.w = orientation.get("w") 

    pose_stamped.pose.covariance = covariance

    return pose_stamped 

def current_time(utc=False, format='%Y-%m-%d %H:%M%S'):

    """
    Returns the current time in the provided format,
    or 'Y-m-d H:M:S' if no format is passed in.

    Args:
        format (str): the desired date format to use for the datetime.
            Defaults to %Y-%m-%d %H:%M%S if not passed.
        utc (bool): whether to return the current time in UTC or local

    Returns:
        str: the datetime formatted as a string
    """

    if utc:
        now = datetime.datetime.utcnow()
    else:
        now = datetime.datetime.now()

    return datetime.datetime.strftime(now, format)


def current_time_epoch(utc=False):

    """
    Returns the current time in seconds since the epoch.

    Args:
        utc (bool): whether to return the current time in UTC or local
    Returns:
        int: the current time in seconds since the epoch.
    """

    if not utc:
        current_offset = time.gmtime().tm_hour - time.localtime().tm_hour
        return int(time.time()) - (current_offset * 3600000)
    return int(time.time())

