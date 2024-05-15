import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2 as cv
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
        cvImg = cv.flip(
            cvImg, 0
        )  # flip the image vertically, since coppelia publishes weird images

        grayImg = cv.cvtColor(cvImg, cv.COLOR_BGR2GRAY)  # convert to gray

        edges = cv.Canny(
            grayImg, 50.0, 150.0, np.ndarray((3, 3)), 3
        )  # find and detect edges

        blurred = cv.blur(edges, (3, 3), 0)  # blur the image

        hough = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, 1.2, 1)
        if not (hough is None):
            copy = cvImg.copy()
            for circle in hough:
                print(len(hough))
                copy = cv.circle(
                    copy,
                    (int(circle[0][0]), int(circle[0][1])),
                    int(circle[0][2]),
                    (0, 0, 250),
                )
            cv.imshow("drawing", cvImg)
        print(hough)
        cv.imshow("edges", edges)  # show the image
        # cv.imshow("blurred", hough)  # show the image
        cv.waitKey(3)


def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
