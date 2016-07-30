class AngularSmoother(object):

    def __init__(self):
        self.ANGULAR_FRICTION = 0.98
        self.MAX_VEL = 3.0
        self.MIN_DIFF = 0.05
        self.MIN_VEL = 0.05
        self.completed = False

    def smooth(self, twist_msg):
        angular_z = twist_msg.angular.z
        if abs(angular_z) <= self.MIN_VEL:
            self.completed = True
            angular_z = 0.0
        else:
            # angular_vel += (self.targ_angular_vel - angular_vel) * 0.4

            # Check if above max vel and normalize it if so
            if abs(angular_z) > self.MAX_VEL:
                angular_z = self.MAX_VEL if angular_z > 0.0 else -self.MAX_VEL

            angular_z *= self.ANGULAR_FRICTION

        twist_msg.angular.z = angular_z

        # Smooth out linear motion to an easing stop
        twist_msg.linear.x /= 1.025 
        twist_msg.linear.y /= 1.025 

        return twist_msg
