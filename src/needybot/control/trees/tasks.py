import json

import rospy

import pi_trees_lib.pi_trees_lib as pt 

from needybot.control.trees.base import Spawner, Task 
from needybot.lib.logger import *

from needybot_srvs.srv import *

class TaskRunner(Task):

    def __init__(self, blackboard):
        """
        Tree task responsible for keeping tasks running. If task is running, tree will stop here
        and not continue through to the spawner. However, if task is completed or failed, tree will
        continue through to the TaskSpawner and spawn a new task.
        """

        super(TaskRunner, self).__init__("task_runner", blackboard)

    def run(self):

        """
        Run sequence of this behavior tree task.

        First checks if a current task is set. If not, it sets a status of FAILURE so it
        will continue through to execute it's sibling, the TaskSpawner.

        If a current task exists, it proceeds to check that tasks status. If completed or failed,
        the runner returns FAILURE, so the tree proceeds to execute it's sibliing, the TaskSpawner.

        If not COMPLETED or FAILED, it return SUCCESS with pre-empts this branch of the tree and
        prevents any siblings from being executed.
        """

        if not self.blackboard.current_task_name:
            return pt.TaskStatus.FAILURE

        try:
            res = self.blackboard.status_proxy()
            if res.status == TaskStatusRequest.COMPLETED:
                return pt.TaskStatus.FAILURE
            elif res.status == TaskStatusRequest.FAILED:
                return pt.TaskStatus.FAILURE
            else:
                return pt.TaskStatus.SUCCESS
        except Exception as e:
            rospy.logerr("Unable to check task status {}".format(e))
            return pt.TaskStatus.FAILURE


class TaskSpawner(Spawner):

    def __init__(self, name, blackboard):

        """
        Spawner responsible for meet_person tasks.
        """

        super(TaskSpawner, self).__init__(name, blackboard)

    def timer_handler(self, evt=None):

        """
        Timer handler for spawning new tasks. Ensures tasks aren't spawned immediately
        and that we wait x-seconds before loading new ones.

        Args:
            evt(TimerEvent): rospy TimerEvent instance
        """

        # Calls LoadTask proxy on next task in queue
        results = self.blackboard.server.serve_task()
        if results:
            (self.blackboard.current_task_name, self.blackboard.current_task_id) = (results[0], results[1])

        self.reset()
