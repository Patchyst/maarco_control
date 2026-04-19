import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Float64MultiArray, String
import py_trees

from maarco_bt.behaviours.conditions import NotStuck, YawErrorAbove, IsWetSand, IsDrySand
from maarco_bt.behaviours.actions import SetGains, TryAltLocomotion, CallForHelp, SetModeScrew, SetModeCrab

from serial_interfaces.msg import SensorData

"""
Subscribes:
    /is_stuck     (std_msgs/Bool)
    /terrain_id   (std_msgs/String)  "wet_sand" | "dry_sand"
    /current_yaw  (std_msgs/Float64)

Publishes:
    /pd_gains     (std_msgs/Float64MultiArray)

colcon build --packages-select maarco_bt
source install/setup.bash
ros2 run maarco_bt heading_bt_node

# SetGains(low) kp=0.4
ros2 topic pub /current_yaw std_msgs/msg/Float64 "{data: 5.0}" --rate 10

# SetGains(mid) kp=1.0
ros2 topic pub /current_yaw std_msgs/msg/Float64 "{data: 15.0}" --rate 10

# SetGains(high) kp=2.0
ros2 topic pub /current_yaw std_msgs/msg/Float64 "{data: 45.0}" --rate 10

# not stuck
ros2 topic pub /is_stuck std_msgs/msg/Bool "{data: false}" --rate 10

# stuck
ros2 topic pub /is_stuck std_msgs/msg/Bool "{data: true}" --rate 10

# wet sand
ros2 topic pub /terrain_id std_msgs/msg/String "{data: wet_sand}" --rate 10

# dry sand
ros2 topic pub /terrain_id std_msgs/msg/String "{data: dry_sand}" --rate 10
"""

class HeadingBTNode(Node):

    def __init__(self):
        super().__init__("heading_bt")

        self.is_stuck = False
        self.needs_help = False
        self.terrain = "wet_sand"
        self.mode = "screw"
        self.current_yaw = 0.0
        self.desired_yaw = 0.0

        # subs
        self.create_subscription(Bool, "/is_stuck", self.stuck_cb, 10)
        self.create_subscription(String, "/terrain_id", self.terrain_cb, 10)

        # This is good for testing, but we need to get current yaw from sensor data.
        #self.create_subscription(Float64, "/current_yaw", self.yaw_cb, 10)
        self.create_subscription(SensorData, "/sensor_data", self.sensor_cb, 10)

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

    def stuck_cb(self, msg):
        self.is_stuck = msg.data

    def terrain_cb(self, msg):
        self.terrain = msg.data

    def yaw_cb(self, msg):
        self.current_yaw = msg.data
    
    def sensor_cb(self, msg):
        self.current_yaw = msg.yaw

    # Build the tree
    def build_tree(self):

        # Recovery Sequence
        recovery_seq = py_trees.composites.Sequence(name="Try recovery", memory=False)
        # Add the try alt locomotion action and not stuck condition
        recovery_seq.add_children([TryAltLocomotion(self), NotStuck(self, name="Not stuck after recovery?")])

        # Selector condition for checking if we are stuck
        stuck_check = py_trees.composites.Selector(name="Stuck check", memory=False)

        # Check if we are stuck, if so then try the recovery sequence (above), and if that fails call for help
        stuck_check.add_children([NotStuck(self), recovery_seq, CallForHelp(self)])

        # Wet sand branch = screw mode
        wet_sand_seq = py_trees.composites.Sequence(name="Wet sand", memory=False)
        wet_sand_seq.add_children([IsWetSand(self), SetModeScrew(self)])

        # Dry sand branch = crab mode
        dry_sand_seq = py_trees.composites.Sequence(name="Dry sand", memory=False)
        dry_sand_seq.add_children([IsDrySand(self), SetModeCrab(self)])

        terrain_selector = py_trees.composites.Selector(name="Terrain selector", memory=False)
        terrain_selector.add_children([wet_sand_seq, dry_sand_seq])

        # If we arent stuck, then we switch into yaw adjustment towards the goal

        # handle large errors
        large_error_seq = py_trees.composites.Sequence(name="Large error", memory=False)
        large_error_seq.add_children([YawErrorAbove(self, 30.0), SetGains(self, kp=2.0, kd=0.4, label="high")])

        # handle medium errors
        medium_error_seq = py_trees.composites.Sequence(name="Medium error", memory=False)
        medium_error_seq.add_children([YawErrorAbove(self, 10.0), SetGains(self, kp=1.0, kd=0.2, label="mid")])

        gain_selector = py_trees.composites.Selector(name="Gain selector", memory=False)
        gain_selector.add_children([large_error_seq, medium_error_seq, SetGains(self, kp=0.4, kd=0.05, label="low")])

        # root node is just a sequence
        root = py_trees.composites.Sequence(name="Root", memory=False)
        root.add_children([stuck_check, terrain_selector, gain_selector])
        return root

    # Tick the nodes of the tree
    def tick(self):
        self.tree.tick_once()


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