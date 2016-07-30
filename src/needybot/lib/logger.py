import rospy

def compose_log(msg, char="-"):
    msg_len = len(msg)
    border_str = char*8 + char*msg_len
    return (border_str, "{} {} {}".format(char*3, msg, char*3), border_str)

def logerr(msg, char="*"):
    logs = compose_log(msg, char)
    for log in logs:
        rospy.logerr(log)

def loginfo(msg, char="-"):
    logs = compose_log(msg, char)
    for log in logs:
        rospy.loginfo(log)

def logwarn(msg, char="="):
    logs = compose_log(msg, char)
    for log in logs:
        rospy.logwarn(log)
