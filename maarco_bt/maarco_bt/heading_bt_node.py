import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Float64MultiArray
import py_trees

from maarco_bt.behaviours.conditions import NotStuck, YawErrorAbove
from maarco_bt.behaviours.actions import SetGains, TryAltLocomotion, CallForHelp


"""
Subscribes:
    /yaw_error  (std_msgs/Float64) 
    /is_stuck   (std_msgs/Bool)

Publishes:
    /pd_gains   (std_msgs/Float64MultiArray)

colcon build --packages-select maarco_bt
source install/setup.bash
ros2 run maarco_bt heading_bt_node

# SetGains(low) kp=0.4
ros2 topic pub /yaw_error std_msgs/msg/Float64 "{data: 5.0}" --rate 10

# SetGains(mid) kp=1.0
ros2 topic pub /yaw_error std_msgs/msg/Float64 "{data: 15.0}" --rate 10

# SetGains(high) kp=2.0
ros2 topic pub /yaw_error std_msgs/msg/Float64 "{data: 45.0}" --rate 10

# not stuck
ros2 topic pub /is_stuck std_msgs/msg/Bool "{data: false}" --rate 10

# stuck
ros2 topic pub /is_stuck std_msgs/msg/Bool "{data: true}" --rate 10

"""
class HeadingBTNode(Node):

    def __init__(self):
        super().__init__("heading_bt")

  
        self.yaw_error  = 0.0
        self.is_stuck  = False

        # subs
        self.create_subscription(Float64, "/yaw_error", self.yaw_cb, 10)
        self.create_subscription(Bool,    "/is_stuck",  self.stuck_cb, 10)

        # pubs
        self.gains_pub = self.create_publisher(Float64MultiArray, "/pd_gains", 10)

        # build and start tree
        self.tree = self.build_tree()
        py_trees.logging.level = py_trees.logging.Level.INFO

        tick_hz = 20.0
        self.create_timer(1.0 / tick_hz, self.tick)
        self.get_logger().info(f"Heading BT running at {tick_hz} Hz.")

        # do we need help?
        self.needs_help = False

    # Callbacks

    def yaw_cb(self, msg):
        self.yaw_error = msg.data

    def stuck_cb(self, msg):
        self.is_stuck = msg.data

    # Build the tree
    def build_tree(self):

        # Recovery Sequence
        recovery_seq = py_trees.composites.Sequence( name="Try recovery", memory=False)
        # Add the try alt locomotion action and not stuck condition
        recovery_seq.add_children([TryAltLocomotion(self), NotStuck(self, name="Not stuck after recovery?"),])

        # Selector condition for checking if we are stuck
        stuck_check = py_trees.composites.Selector(name="Stuck check", memory=False)

        # Check if we are stuck, if so then try the recovery sequence (above), and if that fails call for help
        stuck_check.add_children([NotStuck(self), recovery_seq, CallForHelp(self),])

        # If we arent stuck, then we switch into yaw adjustment towards the goal
        
        # handle large errors
        large_error_seq = py_trees.composites.Sequence(name="Large error", memory=False)
        large_error_seq.add_children([YawErrorAbove(self, 30.0), SetGains(self, kp=2.0, kd=0.4, label="high"),])
        
        # handle medium errors
        medium_error_seq = py_trees.composites.Sequence(name="Medium error", memory=False)
        medium_error_seq.add_children([YawErrorAbove(self, 10.0), SetGains(self, kp=1.0, kd=0.2, label="mid"),])

        gain_selector = py_trees.composites.Selector(name="Gain selector", memory=False)
        gain_selector.add_children([large_error_seq,medium_error_seq, SetGains(self, kp=0.4, kd=0.05, label="low"),])

        # root node is just a sequence
        root = py_trees.composites.Sequence(name="Root", memory=False)
        root.add_children([stuck_check, gain_selector])
        return root

    # Tick the nodes of the tree

    def tick(self):
        self.tree.tick_once()


# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = HeadingBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()