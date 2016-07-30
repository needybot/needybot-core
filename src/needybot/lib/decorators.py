import rospy
from rospy.exceptions import ROSInterruptException

def handle_shutdown_exception(func):
    def wrapped(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ROSInterruptException as e:
            if str(e) == 'rospy shutdown':
                rospy.logwarn('Service failed to connect due to a shutdown interrupt.')
            else:
                raise e
    return wrapped


