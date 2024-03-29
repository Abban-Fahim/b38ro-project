def get_robot(name):
    # Construct the file path to the URDF file based on the robot name
    file_path = "../assets/urdf/" + name + "/" + name + ".urdf"

    # Import the required robot class from Pykin based on the robot name
    if name == "KR7108-URDF":
        from pykin.robots.single_arm import SingleArm

    robot = SingleArm(file_path)  # Initialize and return the robot object
    return robot


# Get the robot object for KR7108-URDF
robot = get_robot("KR7108-URDF")

# Display information about the robot
robot.show_robot_info()
