ros2 control switch_controllers --deactivate joint_trajectory_controller robotiq_gripper_controller

ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger

ros2 control switch_controllers --activate joint_trajectory_controller robotiq_gripper_controller

ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
    ]
  }" -1
