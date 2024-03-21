import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class for creating ROS nodes

# Import message types for communication
from std_msgs.msg import String, Float32MultiArray

# Import a function to retrieve package paths
from ament_index_python import get_package_prefix

# Import the Chain class from ikpy library for inverse kinematics
import ikpy.chain


# Define a class for our ROS node
class MinimalPublisher(Node):

    # Constructor method
    def __init__(self):
        super().__init__("joints")  # Initialize the node with name "joints"
        
        # Create a publisher for target angles
        self.target_angles = self.create_publisher(
            Float32MultiArray, "target_angles", 10
        )
        
        # Create a subscription to receive web angles
        self.create_subscription(
            Float32MultiArray, "web_angles", self.move_to_angles, 10
        )

        # Get the package directory path
        pkg_dir = get_package_prefix("joints")
        
        # Load robot chain from URDF file
        self.robot = ikpy.chain.Chain.from_urdf_file(
            pkg_dir + "/../../../coppelia/GEN3-LITE.urdf",
            ["BASE"],
            active_links_mask=[True, True, True, True, True, True, True, False],
        )

    # Method to calculate joint angles and publish target angles
    def move_to_angles(self, msg: Float32MultiArray):
        # Calculate inverse kinematics for the received web angles
        angles = self.robot.inverse_kinematics(msg.data)
        
        # Create a new message for publishing
        msg = Float32MultiArray()
        
        # Set the data field of the message to the calculated joint angles
        msg.data = angles[1:7].tolist()
        
        # Publish the calculated joint angles
        self.target_angles.publish(msg)


# Main function to initialize the node and start the event loop
def main():
    rclpy.init()  # Initialize the ROS 2 client library

    minimal_publisher = MinimalPublisher()  # Create an instance of MinimalPublisher

    rclpy.spin(minimal_publisher)  # Start the event loop

    # Explicitly destroy the node
    minimal_publisher.destroy_node()

    rclpy.shutdown()  # Shutdown the ROS 2 client library


# Entry point of the script
if __name__ == "__main__":
    main()  # Call the main function to start the node
