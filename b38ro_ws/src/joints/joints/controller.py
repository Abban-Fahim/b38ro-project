import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32


class Controller(Node):
    def __init__(self):
        super().__init__("ds4_controller")

        self.log = self.get_logger()

        self.create_subscription(Joy, "/joy", self.controller_cb, 10)

        # self.twist_topic = self.create_publisher(
        #     Twist, "/twist_controller/commands", 10
        # )

        # Subscribe to current cartesian position and angles of robot
        self.create_subscription(Pose, "/robot_position", self.robot_pos_cb, 10)
        self.initial_set = False
        self.arm_pose = Pose()
        # Publisher to send new cartesian positions
        self.arm_pose_pub = self.create_publisher(Pose, "/cart_pose", 10)

        self.gripper_topic = self.create_publisher(Float32, "gripper_pose", 10)

        self.last_publish = self.get_clock().now()

    def robot_pos_cb(self, msg):
        if not self.initial_set:
            self.arm_pose = msg
            self.initial_set = True

    def controller_cb(self, msg: Joy):
        # Open and close the gripper
        # Open if X is pressed
        if msg.buttons[0] == 1:
            val = Float32()
            val.data = 0.0
            self.gripper_topic.publish(val)
        # Close if Circle is pressed
        elif msg.buttons[1] == 1:
            val = Float32()
            val.data = 0.8
            self.gripper_topic.publish(val)
        elif msg.buttons[2] == 1:
            self.arm_pose.orientation.x = math.pi
            self.arm_pose.orientation.y = self.arm_pose.orientation.z = 0.0
            self.arm_pose_pub.publish(self.arm_pose)

        # Only publish position once every second, to ensure arm doesnt jerk too much
        curr_time = self.get_clock().now()
        time_diff = curr_time - self.last_publish
        if time_diff.nanoseconds > 500000000:
            self.last_publish = curr_time
            self.arm_pose.position.x += -msg.axes[0] * 0.2
            self.arm_pose.position.y += msg.axes[1] * 0.2
            self.arm_pose.position.z += msg.axes[4] * 0.2

            # Send new arm position to move to
            self.arm_pose_pub.publish(self.arm_pose)

    # Old twist controller
    # def controller_cb(self, msg: Joy):
    #     if msg.buttons[0] == 1:
    #         val = Float32()
    #         val.data = 0.0
    #         self.gripper_topic.publish(val)
    #     elif msg.buttons[1] == 1:
    #         val = Float32()
    #         val.data = 0.8
    #         self.gripper_topic.publish(val)

    #     twist = Twist()
    #     twist.linear.x = msg.axes[0]
    #     twist.linear.y = msg.axes[1]

    #     twist.angular.x = msg.axes[3]
    #     twist.angular.z = msg.axes[4]
    #     # Unpressed is 1.0, pressed down fully is -1.0
    #     l2 = msg.axes[2]
    #     r2 = msg.axes[5]
    #     twist.linear.z = 0.5 * (l2 - r2)
    #     print(twist)
    #     self.twist_topic.publish(twist)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
