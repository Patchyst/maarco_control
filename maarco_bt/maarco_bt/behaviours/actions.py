import py_trees
from std_msgs.msg import Float64MultiArray

# Publishes kp/kd to /pd_gains and returns SUCCESS immediately.
# kp/kd are hardcoded
# This action 
class SetGains(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, kp, kd, label):
        super().__init__(name=f"SetGains({label})")
        self.ros_node = ros_node
        self.kp = kp
        self.kd = kd

    def update(self):
        msg = Float64MultiArray()
        msg.data = [self.kp, self.kd]
        self.ros_node.gains_pub.publish(msg)
        self.logger.info(f"Yaw Control. Published gains kp={self.kp} kd={self.kd}")
        return py_trees.common.Status.SUCCESS

# Returns RUNNING while in progress, SUCCESS when the attempt is done.
# update method will contain recovery logic
class TryAltLocomotion(py_trees.behaviour.Behaviour):

    def __init__(self, ros_node, name="TryAltLocomotion", max_ticks=200):
        super().__init__(name)
        self.ros_node = ros_node
        self.max_ticks = max_ticks
        self.ticks = 0

    def initialise(self):
        self.ticks=0
        self.logger.info("Starting alt locomotion attempt...")

    def update(self):
        self.ticks += 1
        if self.ticks < self.max_ticks:
            self.logger.info("Alt locomotion in process")
            return py_trees.common.Status.RUNNING
        self.logger.info("Alt locomotion attempt complete.")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # make sure to terminate locomotion method when this method is reached, as we are now "unstuck"
        self.logger.info(f"Locomotion terminated with status: {new_status}")

# Get help from operator
# Returns RUNNING indefinitely to block the tree.
class CallForHelp(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, name="CallForHelp"):
        super().__init__(name)
        self.ros_node = ros_node

    # 
    def initialise(self):
        # TODO: insert code for calling help from operator  
        self.needs_help  = True
        self.logger.warning("STUCK: calling for help.")

    def update(self):
        self.ros_node.needs_help = True
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.needs_help = False
        self.logger.info(f"Calling for help terminated with status: {new_status}")