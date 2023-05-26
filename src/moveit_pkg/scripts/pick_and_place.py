import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys


# Initialize the MoveIt commander and get the arm and gripper groups
moveit_commander.roscpp_initialize(sys.argv)
arm_group = moveit_commander.MoveGroupCommander("arm")
#gripper_group = moveit_commander.MoveGroupCommander("gripper")

# Define the pick and place poses as PoseStamped messages
pick_pose = PoseStamped()
pick_pose.header.frame_id = "base"
pick_pose.pose.position.x = 0.5
pick_pose.pose.position.y = 0.0
pick_pose.pose.position.z = 0.5
pick_pose.pose.orientation.x = 0.0
pick_pose.pose.orientation.y = 0.707
pick_pose.pose.orientation.z = 0.0
pick_pose.pose.orientation.w = 0.707

place_pose = PoseStamped()
place_pose.header.frame_id = "base"
place_pose.pose.position.x = 0.5
place_pose.pose.position.y = 0.5
place_pose.pose.position.z = 0.5
place_pose.pose.orientation.x = 0.0
place_pose.pose.orientation.y = 0.0
place_pose.pose.orientation.z = 0.0
place_pose.pose.orientation.w = 1.0

# Compute the joint values for the pick pose using inverse kinematics
pick_joint_values = arm_group.get_current_joint_values()
pick_joint_values = arm_group.compute_ik(pick_pose, "gripper")
gripper_joint_values = [0.02]

# Set the joint value targets for the arm and gripper
arm_group.set_joint_value_target(pick_joint_values)
#gripper_group.set_joint_value_target(gripper_joint_values)

# Plan and execute the pick motion
arm_plan = arm_group.plan()
gripper_plan = gripper_group.plan()
arm_group.execute(arm_plan)
#gripper_group.execute(gripper_plan)

# Compute the joint values for the place pose using inverse kinematics
place_joint_values = arm_group.get_current_joint_values()
place_joint_values = arm_group.compute_ik(place_pose, "gripper")
gripper_joint_values = [0.0]

# Set the joint value targets for the arm and gripper
arm_group.set_joint_value_target(place_joint_values)
#gripper_group.set_joint_value_target(gripper_joint_values)

# Plan and execute the place motion
arm_plan = arm_group.plan()
#gripper_plan = gripper_group.plan()
arm_group.execute(arm_plan)
#gripper_group.execute(gripper_plan)

