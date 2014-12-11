#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import baxter_interface

## First initialize moveit_commander and rospy.
print "============ Starting tutorial setup"
rospy.init_node('master', anonymous=True)
moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("left_arm")
group.set_goal_position_tolerance(0.005)
group.set_goal_orientation_tolerance(0.01) 

print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
pose_target.orientation.w = 0.0
pose_target.position.x = 0.5
pose_target.position.y = 0.5
pose_target.position.z = 0.0
group.set_pose_target(pose_target)

## Now, we call the planner to compute the plan
## and visualize it if successful
## Note that we are just planning, not asking move_group 
## to actually move the robot
plan1 = group.plan()

raw_input('Plan Ready, Press Enter to go to position 1')
group.go()
group.clear_pose_targets()

########################

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
pose_target.orientation.w = 0.0
pose_target.position.x = 0.5
pose_target.position.y = 0.5
pose_target.position.z = 0.0+ 0.1
group.set_pose_target(pose_target)

## Now, we call the planner to compute the plan
## and visualize it if successful
## Note that we are just planning, not asking move_group 
## to actually move the robot
plan1 = group.plan()

raw_input('Plan Ready, Press Enter to go to position 2')
group.go()
group.clear_pose_targets()


pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
pose_target.orientation.w = 0.0
pose_target.position.x = 0.5
pose_target.position.y = 0.5
pose_target.position.z = 0.0
group.set_pose_target(pose_target)

## Now, we call the planner to compute the plan
## and visualize it if successful
## Note that we are just planning, not asking move_group 
## to actually move the robot
plan1 = group.plan()

raw_input('Plan Ready, Press Enter to go to position 1')
group.go()
group.clear_pose_targets()




# Arm_left = baxter_interface.Limb('left')
# Angle = Arm_left.joint_angles()
# CurrentJointAngle = [ Angle['left_s0'], Angle['left_s1'], Angle['left_e0'], Angle['left_e1'], 
#                       Angle['left_w0'], Angle['left_w1'], Angle['left_w2']  ]  

# TargetJointAngle = CurrentJointAngle
# TargetJointAngle[1] = TargetJointAngle[1] - 0.1
# group.set_joint_value_target(TargetJointAngle)

# plan2 = group.plan()

# raw_input('Plan Ready, Press Enter to move left_s1 up')
# group.go()





## Cartesian Paths
## ^^^^^^^^^^^^^^^
## You can plan a cartesian path directly by specifying a list of waypoints 
## for the end-effector to go through.

Arm_left = baxter_interface.Limb('left')
# Angle = Arm_left.joint_angles()
waypoints = []

# start with the current state
POSE = Arm_left.endpoint_pose()
wpose = geometry_msgs.msg.Pose()
wpose.position.x = POSE['position'].x
wpose.position.y = POSE['position'].y
wpose.position.z = POSE['position'].z
wpose.orientation.x = 0.0
wpose.orientation.y = 1.0
wpose.orientation.z = 0.0
wpose.orientation.w = 0.0

waypoints.append(copy.deepcopy(wpose))

# first orient gripper and move forward (+x)

wpose.position.z = waypoints[0].position.z + 0.1
waypoints.append(copy.deepcopy(wpose))

# second move to dirise 10 cm
wpose.position.x = 0.5
wpose.position.y = 0.2
wpose.position.z = 0.1 + 0.1
waypoints.append(copy.deepcopy(wpose))

# third move to the desired position
wpose.position.z = 0.1 
waypoints.append(copy.deepcopy(wpose))

## We want the cartesian path to be interpolated at a resolution of 1 cm
## which is why we will specify 0.01 as the eef_step in cartesian
## translation.  We will specify the jump threshold as 0.0, effectively
## disabling it.
(plan3, fraction) = group.compute_cartesian_path(
                           waypoints,   # waypoints to follow
                           0.01,        # eef_step
                           0.0)         # jump_threshold
                           
raw_input('Plan Ready, Press Enter to go')
group.execute(plan3) 
#group.go()
#group.clear_pose_targets()

raw_input('exit')

## When finished shut down moveit_commander.
moveit_commander.roscpp_shutdown()

## END_TUTORIAL

print "============ STOPPING"



#  !!!!!!!!!!!!!!!!!!!!!!!!
#