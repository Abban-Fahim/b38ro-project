import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray
import cv2 as cv
import cv_bridge
import numpy as np


class Vision(Node):
    def __init__(self):
        super().__init__("ttt_vision")

        self.create_subscription(Image, "/camera/image/raw", self.camera_cb, 10)
        self.bridge = cv_bridge.CvBridge()

        # topics to interact with the game logic
        self.create_subscription(
            Int32MultiArray, "/board_state", self.recieve_board, 10
        )
        self.movePublisher = self.create_publisher(Int32, "/human_move", 10)

        self.boardState = np.zeros((9,))
        self.boardStateRecieved = None

        self.debug = False

    def camera_cb(self, msg: Image):
        cvImg = self.bridge.imgmsg_to_cv2(
            msg, "bgr8"
        )  # convert ROS Image to ndarray (compatible with opencv)
        cvImg = cv.flip(
            cvImg, 0
        )  # flip the image vertically, since coppelia publishes weird images

        grayImg = cv.cvtColor(cvImg, cv.COLOR_BGR2GRAY)  # convert to gray

        eroded = cv.erode(grayImg, np.ones((6, 6)))

        edges = cv.Canny(
            eroded, 50.0, 150.0, np.ones((3, 3)), 3
        )  # find and detect edges

        # Detect lines using Hough transformation
        lines = cv.HoughLines(edges, 1.0, np.pi / 2, 100)

        hLines = []
        vLines = []
        if lines is not None:
            # Firstly filter out the lines that are very close to each other
            # Ideally, only 4 lines should be in the doc
            lines = self.unify_lines(lines)
            if len(lines) == 4:
                for line in lines:
                    rho = line[0][0]
                    theta = line[0][1]
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                    pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))

                    # Add cartesian parameters of line to an array whether its vertical or horizontal
                    if abs(theta) < 1e-7:
                        vLines.append(int(rho))
                    else:
                        hLines.append(int(rho))
                    # Draw purple lines showing the grid lines of the boar
                    cv.line(cvImg, pt1, pt2, (255, 0, 255), 1, cv.LINE_AA)

                    if len(hLines) == 2 and len(vLines) == 2:
                        width = hLines[1] - hLines[0]
                        height = vLines[1] - vLines[0]
                        startX = hLines[0] - width
                        startY = vLines[0] - height

                        segs = np.ndarray((3, 3), cv.Mat)
                        for row in range(3):
                            for cell in range(3):
                                seg = grayImg[
                                    startX : startX + width, startY : startY + height
                                ]
                                segs[row, cell] = seg
                                self.boardState[cell * 3 + row] = (
                                    self.check_major_colour(seg)
                                )
                                startX += width
                            startY += height
                            startX = hLines[0] - width
                        if self.boardStateRecieved is not None:
                            diff = self.boardState - self.boardStateRecieved
                            self.get_logger().info(str(diff))
                            for i in range(9):
                                if diff[i] == 1:
                                    msg = Int32()
                                    msg.data = i
                                    self.movePublisher.publish(msg)
                                    i = 9

        if self.debug:
            cv.imshow("edges", edges)  # show the image
            cv.imshow("eroded", eroded)  # show the image
            cv.imshow("drawing", cvImg)
            cv.imshow("gray", grayImg)
            cv.waitKey(3)

    def recieve_board(self, msg: Int32MultiArray):
        self.boardStateRecieved = np.array(msg.data)

    # Function that unifies and centres two very similiar lines
    # based on thresholds for the polar parameters
    # Credit goes to: https://stackoverflow.com/a/76918385/14151834/
    def unify_lines(self, lines, rho_threshold=3, theta_threshold=np.pi / 180 * 10):
        unified_lines = []
        for line in lines:
            for rho, theta in line:
                if not unified_lines:
                    unified_lines.append((rho, theta))
                else:
                    matched = False
                    for u_rho, u_theta in unified_lines:
                        if (
                            abs(u_rho - rho) < rho_threshold
                            and abs(u_theta - theta) < theta_threshold
                        ):
                            average_rho = (u_rho + rho) / 2
                            average_theta = (u_theta + theta) / 2
                            unified_lines[unified_lines.index((u_rho, u_theta))] = (
                                average_rho,
                                average_theta,
                            )
                            matched = True
                            break
                    if not matched:
                        unified_lines.append((rho, theta))
        return np.array([[line] for line in unified_lines], dtype=np.float32)

    def check_major_colour(self, img: cv.Mat):
        grayCount = 0
        whiteCount = 0
        for i in img:
            for j in i:
                if j == 115:
                    grayCount += 1
                elif j == 255:
                    whiteCount += 1

        if grayCount >= 200 or whiteCount >= 200:
            if grayCount >= whiteCount:
                return 1
            else:
                return 2
        else:
            return 0


def main():
    rclpy.init()
    node = Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
