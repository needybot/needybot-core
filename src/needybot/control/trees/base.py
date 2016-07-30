import rospy

import pi_trees_lib.pi_trees_lib as pt 

class Task(pt.Task):

    def __init__(self, name, blackboard):
        super(Task, self).__init__(name)
        self.blackboard = blackboard


class Spawner(Task):

    def __init__(self, name, blackboard):

        """
        Tree task responsible for spawning new tasks. Once a task has a status of completed of failure
        this branch of the tree is executed. The timer waits for 5 seconds before sending the final 
        complete signal to the running task and spawning a new one.

        Args:
            name(str):              Name of this behavior tree task for identification.
            blackboard(Blackboard): Behavior tree blackboard that stores global vars.
        """

        super(Spawner, self).__init__(name, blackboard)
        self.timer = None
        self.spawning = False
        self.spawn_delay = rospy.get_param("~spawn-delay", 0.0)

    def reset(self):

        """
        Reset the spawn timer and spawning flags.
        """

        if self.timer:
          self.timer.shutdown()

        self.timer = None
        self.spawning = False

    def run(self):

        """
        Pi Trees run callback that gets called when a previous task is completed.
        Responsible for spawning up next task and passing data
        b/t previous task and new one.
        """

        # sleep to make sure completion audio and expression don't get
        # overrun by the next task
        if not self.spawning:
            self.spawning = True
            if self.spawn_delay > 0:
              self.timer = rospy.Timer(rospy.Duration(self.spawn_delay), self.timer_handler, oneshot=True)
            else:
              self.timer_handler()

        return pt.TaskStatus.RUNNING
