#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import copy


def IK_MoveIt(MoveIt_arm, StartPosition, MiddlePosition,EndPosition , Accuracy):

    waypoints = []  

    wpose = Pose()
    wpose.orientation.x = 0.0
    wpose.orientation.y = 1.0
    wpose.orientation.z = 0.0
    wpose.orientation.w = 0.0

    # first point
    wpose.position.x = StartPosition[0]
    wpose.position.y = StartPosition[1]
    wpose.position.z = StartPosition[2]
    waypoints.append(copy.deepcopy(wpose))

    # Middle Point (if existed)
    if MiddlePosition != False :
        Number = len(MiddlePosition)/3 - 1
        for i in range(Number):
            wpose.position.x = MiddlePosition[i*3]
            wpose.position.y = MiddlePosition[i*3+1]
            wpose.position.z = MiddlePosition[i*3+2]
            waypoints.append(copy.deepcopy(wpose))

    # End point
    wpose.position.x = EndPosition[0]
    wpose.position.y = EndPosition[1]
    wpose.position.z = EndPosition[2]
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = MoveIt_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               Accuracy,        # eef_step
                               0.0)         # jump_threshold
    
    # Execute the plan         
    #raw_input('Press Enter to go')            
    MoveIt_arm.execute(plan) 

def IK_calculate(MoveIt_arm,Position,Quaternion):

    # MoveIt_arm : MoveIt_left_arm  or MoveIt_right_arm

    #Start a node
    #rospy.init_node('moveit_node')

    #Create a goal pose for the left/right arm
    arm_goal_pose = PoseStamped()
    arm_goal_pose.header.frame_id = "base"

    #x, y, and z position
    arm_goal_pose.pose.position.x = Position[0]
    arm_goal_pose.pose.position.y = Position[1]
    arm_goal_pose.pose.position.z = Position[2]
    self
    #Orientation as a quaternion
    arm_goal_pose.pose.orientation.x = Quaternion[0]
    arm_goal_pose.pose.orientation.y = Quaternion[1]
    arm_goal_pose.pose.orientation.z = Quaternion[2]
    arm_goal_pose.pose.orientation.w = Quaternion[3]

    #Set the goal state to the pose you just defined
    MoveIt_arm.set_pose_target(arm_goal_pose)

    #Set the start state for the left/right arm
    MoveIt_arm.set_start_state_to_current_state()

    #Plan a path
    IK_plan = MoveIt_arm.plan()

    return IK_plan 


def IK_Execute(MoveIt_arm,IK_plan):
    #Execute the plan
    MoveIt_arm.execute(IK_plan)



def FK_calculate(MoveIt_arm,JointAngle):

    
    
    MoveIt_arm.set_joint_value_target(JointAngle)

    #Set the start state for the left/right arm
    MoveIt_arm.set_start_state_to_current_state()

    #Plan a path
    IK_plan = MoveIt_arm.plan()

    MoveIt_arm.execute(IK_plan)


    # .65376956364916, 0.15558032315443168