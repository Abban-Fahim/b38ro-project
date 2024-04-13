import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import cv_bridge


class Vision(Node):
    def __init__(self):
        super().__init__("ttt_vision")

        self.create_subscription(Image, "/camera/image/raw", self.camera_cb, 10)
        self.bridge = cv_bridge.CvBridge()

    def camera_cb(self, msg: Image):
        cvImg = self.bridge.imgmsg_to_cv2(
            msg, "bgr8"
        )  # convert ROS Image to ndarray (compatible with opencv)
        cvImg = cv2.flip(
            cvImg, 0
        )  # flip the image vertically, since coppelia publishes weird images
        grayImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY)
        cv2.imshow("bru", grayImg)  # show the ima
        cv2.waitKey(3)


def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
