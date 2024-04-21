import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import cv_bridge
import numpy as np


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

        grayImg = cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY)  # convert to gray

        blurred = cv2.blur(grayImg, (3, 3), 0)  # blur the image

        edges = cv2.Canny(
            grayImg, 50.0, 150.0, np.ndarray((3, 3)), 3
        )  # find and detect edges

        blurredEdges = cv2.GaussianBlur(edges, (3, 3), 2, sigmaY=2)

        print(edges)
        cv2.imshow("edges", edges)  # show the image
        cv2.imshow("blurred", blurredEdges)  # show the image
        cv2.waitKey(3)


def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
