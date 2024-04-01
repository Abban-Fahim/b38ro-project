import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Controller(Node):
    def __init__(self):
        super().__init__("ds4_controller")

        self.create_subscription(Joy, "/joy", self.controller_cb, 10)
        self.twist_topic = self.create_publisher(
            Twist, "/twist_controller/commands", 10
        )

        self.gripper_topic = self.create_publisher(Float32, "gripper_pose", 10)

    def controller_cb(self, msg: Joy):
        if msg.buttons[0] == 1:
            val = Float32()
            val.data = 0.0
            self.gripper_topic.publish(val)
        elif msg.buttons[1] == 1:
            val = Float32()
            val.data = 0.8
            self.gripper_topic.publish(val)

        twist = Twist()
        twist.linear.x = msg.axes[0]
        twist.linear.y = msg.axes[1]

        twist.angular.x = msg.axes[3]
        twist.angular.z = msg.axes[4]
        # Unpressed is 1.0, pressed down fully is -1.0
        l2 = msg.axes[2]
        r2 = msg.axes[5]
        twist.linear.z = 0.5 * (l2 - r2)
        print(twist)
        self.twist_topic.publish(twist)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
