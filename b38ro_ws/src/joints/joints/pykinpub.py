# Import necessary libraries for ROS 2
import rclpy  # Import the ROS 2 Python library
from rclpy.node import Node  # Import the Node class for creating ROS nodes
from std_msgs.msg import String, Float32MultiArray  # Import message types for communication
from pykin.robots.single_arm import SingleArm  # Import SingleArm robot class from Pykin
from pykin.kinematics.transform import Transform  # Import Transform class from Pykin for defining transformations

# Define a class for our ROS node
class MinimalPublisher(Node):

    # Constructor method
    def __init__(self):
        super().__init__("joints")  # Initialize the node with name "joints"
        
        # Create a publisher for target joint angles
        self.target_angles = self.create_publisher(
            Float32MultiArray, "target_angles", 10
        )
        
        # Create a subscription to receive Cartesian poses
        self.create_subscription(
            Float32MultiArray, "cart_pose", self.move_to_angles, 10
        )

        # Define the file path to the URDF file
        file_path = '../assets/urdf/KR7108-URDF/KR7108-URDF.urdf'
        
        # Load the robot model from the URDF file
        self.robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))
        self.robot.setup_link_name("BASE", "DUMMY")

    # Method to calculate joint angles and publish target angles
    def move_to_angles(self, msg: Float32MultiArray):

        # Initial joint angles
        init_thetas = [0, 0, 0, 0, 0, 0]

        # NOTE
        # Change the line of code above to a correct one.
        # init_thetas = [0, 0, 0, 0, 0, 0]
        # I have a feeling it's wrong.

        # Calculate forward kinematics to get the initial end-effector pose
        self.robot.forward_kin(init_thetas)
        
        # Calculate inverse kinematics for the received Cartesian pose
        angles = self.robot.inverse_kin(init_thetas, msg.data)
    
        # Create a new message for publishing
        msg = Float32MultiArray() 
        
        # Set the data field of the message to the calculated joint angles
        msg.data = angles.tolist()
        
        # Print the calculated joint angles
        print(msg.data)

        # Publish the calculated joint angles
        self.target_angles.publish(msg)


# Main function to initialize the node and start the event loop
def main():
    rclpy.init()  # Initialize the ROS 2 client library
    minimal_publisher = MinimalPublisher()  # Create an instance of MinimalPublisher
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()  # Explicitly destroy the node
    rclpy.shutdown()  # Shutdown the ROS 2 client library


# Entry point of the script
if __name__ == "__main__":
    main()  # Call the main function to start the node
