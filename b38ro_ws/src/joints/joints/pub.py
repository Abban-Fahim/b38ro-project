import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from ament_index_python import get_package_prefix

import ikpy.chain


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("joints")
        self.target_angles = self.create_publisher(
            Float32MultiArray, "target_angles", 10
        )
        self.create_subscription(
            Float32MultiArray, "web_angles", self.move_to_angles, 10
        )

        pkg_dir = get_package_prefix("joints")
        print(pkg_dir)
        self.robot = ikpy.chain.Chain.from_urdf_file(
            pkg_dir + "/../../../kortex_description/gen3_lite/urdf/GEN3-LITE.urdf",
            ["BASE"],
            active_links_mask=[True, True, True, True, True, True, True, False],
        )

    def move_to_angles(self, msg: Float32MultiArray):
        angles = self.robot.inverse_kinematics(msg.data)
        msg = Float32MultiArray()
        msg.data = angles[1:7].tolist()
        print(angles)
        self.target_angles.publish(msg)


def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
