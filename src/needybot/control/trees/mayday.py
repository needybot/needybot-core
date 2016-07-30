import rospy

import base
import pi_trees_lib.pi_trees_lib as pt 

class MaydayRunner(base.Task):

    def __init__(self, blackboard):
        super(MaydayRunner, self).__init__("mayday_runner", blackboard)

    def run(self):

        """
        Simply checks if already in mayday mode. If not returns success and will hit
        the mayday spawner.
        """

        if not self.blackboard.in_mayday:
            return pt.TaskStatus.FAILURE
        else:
            return pt.TaskStatus.SUCCESS


class MaydaySpawner(base.Spawner):

    def __init__(self, blackboard):
        super(MaydaySpawner, self).__init__("mayday_spawner", blackboard)

    def run(self):

        """
        Always returns task status of failure to prevent tree from moving past this point to the
        normal task runner. Only way mayday gets pre-empted is when the check_healthy node returns true,
        which will happen once needybot has enough charge
        """

        super(MaydaySpawner, self).run()
        return pt.TaskStatus.FAILURE

    def timer_handler(self, evt=None):

        """
        Handler for the spawn timer.

        Once fired, we complete any previously running tasks stored in the blackboard.
        It then request a the mayday task from the task_server and sets a flag on the
        blackboard so the task server and behavior nodes can check against it. 
        """

        if self.blackboard.current_task_name is not None:
            res = self.blackboard.step_proxy('complete')

        self.blackboard.current_task_name = self.blackboard.server.home_task()
        self.blackboard.in_mayday = True 

        self.reset()
