#!/usr/bin/env python

import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION

import moveit_commander
import IK

import time
import sys
import tf
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Pose,Vector3,Quaternion
from sensor_msgs.msg import Image

def callback1(msg):
    # Callback for found_markers
    pass

class BuildHouse(object):     
    
    def __init__(self):

        ##### Initialize AR_tag transforms #####
        rospy.init_node('master', anonymous=True)
        self.listener = tf.TransformListener()

        ##### Initialize Inverse Kinematics #####  
        self.Counting = 0
        #Initialize moveit_commander
        moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')
        #Initialize MoveIt for both arms
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.MoveIt_left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.MoveIt_left_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_left_arm.set_goal_orientation_tolerance(0.01) 
        self.MoveIt_right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.MoveIt_right_arm.set_goal_position_tolerance(0.01)
        self.MoveIt_right_arm.set_goal_orientation_tolerance(0.01) 
        ##### Initialize Gripper control #####
        self.Arm_left = baxter_interface.Limb('left')
        self.Arm_right = baxter_interface.Limb('right')
        self.Gripper_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.Gripper_right = baxter_interface.Gripper('right', CHECK_VERSION)

        ##### Define the desired position/orientation for the first block  #####
        #z component dependent on table height
        self.Desired_Position = [ 0.6613, 0.3955, -0.1630]    
        self.Desired_Quaternion = [0.0,1.0,0.0,0.0]
        self.moved_blocks = {}

        ##### Verify robot is enabled  #####
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")


    def GetEndPointPosition(self):
        for i in range(50):
            POSE = self.Arm_left.endpoint_pose()
            EndPoint = [POSE['position'].x ,POSE['position'].y, POSE['position'].z]
        return EndPoint


    def GetTargetBlock(self,detectedMarkers):
        orderedMarkers = []
        for num in detectedMarkers:
            orderedMarkers.append(num)
            if num not in self.moved_blocks:
                self.moved_blocks[num] = 0
        orderedMarkers.sort()
        print orderedMarkers

        chosenMarker = -1
        for i in xrange(len(orderedMarkers)):
            if self.moved_blocks[orderedMarkers[i]] == 0:
                chosenMarker = orderedMarkers[i]
                break

        if chosenMarker != -1:
            self.moved_blocks[chosenMarker] = 1
        return chosenMarker

    def FindBlock(self):
        print "==============="
        print "FIND BLOCK"
        print "==============="
        #Get information about the markers found
        rospy.Subscriber("/demo/found_markers",Int16MultiArray, callback1)
        msg1 = rospy.wait_for_message('/demo/found_markers',Pose)
        print msg1.data
        detectedMarkers = msg1.data

        chosenMarker = self.GetTargetBlock(detectedMarkers)

        if chosenMarker != -1:
            try:
                msg = Pose()
                position = Vector3()
                orientation = Quaternion()
                targetMarker = 'ar_marker_'+str(chosenMarker)
                print "the marker frame chosen", targetMarker               
                (trans,rot) = self.listener.lookupTransform('/base',targetMarker,rospy.Time(0))

                Px = trans[0]
                Py = trans[1]
                Pz = -0.171 #this value dependent on height of table
                Qx = 0.0
                Qy = 1.0
                Qz = 0.0
                Qw = 0.0

                # Return the position/orientation of the target
                self.Block_Position = [Px,Py,Pz] 
                self.Block_Quaternion = [Qx,Qy,Qz,Qw]
                print self.Block_Position 

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        else:
            user_input = raw_input('No blocks left. Please give me more blocks. Press ''Enter'' when ready!!!')
            if user_input == 's':
                moveit_commander.roscpp_shutdown()
            else:   
                self.FindBlock()


    def MoveBlock(self):

        # (1) Approach to Taget Block (10cm above)
        print "MoveBlock Step(1):  Approach to Taget Block (Approximately)"
        
        StartPosition = self.GetEndPointPosition()
        MiddlePosition = [self.Desired_Position[0],self.Desired_Position[1]-0.10 ,self.Desired_Position[2] + 0.10]
        EndPosition = [self.Block_Position[0],self.Block_Position[1] ,self.Block_Position[2] + 0.10]
        Accuracy = 0.03        # Rough
        IK.IK_MoveIt(self.MoveIt_left_arm, StartPosition, MiddlePosition,EndPosition , Accuracy)

        
        # (2) Approach to Target Block
        print "MoveBlock Step(2):  Approach to Target Block"
        StartPosition = self.GetEndPointPosition()
        MiddlePosition = False 
        EndPosition = [self.Block_Position[0],self.Block_Position[1] ,self.Block_Position[2]]
        Accuracy = 0.005       # Accurate
        IK.IK_MoveIt(self.MoveIt_left_arm, StartPosition, MiddlePosition,EndPosition , Accuracy)
        
        # (3) Suction acutator (lift block)
        print "MoveBlock Step(3):  Sucker works (lift block)"
        self.Gripper_control(self.Gripper_left,'close')

        # (4) Move Block to Tower
        print "MoveBlock Step(4):  Move BLocks to House (Approximately)"
        StartPosition = self.GetEndPointPosition()
        MiddlePosition = [ self.Block_Position[0],   self.Block_Position[1] ,         self.Block_Position[2] +0.10,
                           self.Desired_Position[0], self.Desired_Position[1] - 0.10 ,self.Desired_Position[2] +0.10]
        EndPosition = [self.Desired_Position[0],self.Desired_Position[1] ,self.Desired_Position[2] + 0.10 ]
        #if self.Counting <= 2 :
        #    Accuracy = 0.02       # Accurate
        #elif:
        Accuracy = 0.03       # Rough
        IK.IK_MoveIt(self.MoveIt_left_arm, StartPosition, MiddlePosition,EndPosition , Accuracy)

        # (5) Move Block to Desired Position
        print "MoveBlock Step(5):  Move Block to Desired Position"
        StartPosition = self.GetEndPointPosition()
        MiddlePosition = False 
        EndPosition = [self.Desired_Position[0],self.Desired_Position[1] ,self.Desired_Position[2] ]
        Accuracy = 0.005        # Accurate
        IK.IK_MoveIt(self.MoveIt_left_arm, StartPosition, MiddlePosition,EndPosition , Accuracy)
        
        # (6) Suction stops (drop block)
        print "MoveBlock Step(6):  Sucker stops (drop block)"
        self.Gripper_control(self.Gripper_left,'open')
        # (7) Lift 5cm up to avoid collision with the built house
        print "MoveBlock Step(7):  Lift 10cm up to aviod collision with the built house"
        StartPosition = self.GetEndPointPosition()
        MiddlePosition = False   # no middle point
        EndPosition = [self.Desired_Position[0],self.Desired_Position[1] ,self.Desired_Position[2] +0.05]
        Accuracy = 0.005        # Accurate
        IK.IK_MoveIt(self.MoveIt_left_arm, StartPosition, MiddlePosition,EndPosition , Accuracy)


    def UpdateDesiredPosition(self):
        #z component dependent on block size
        self.Desired_Position = [self.Desired_Position[0], self.Desired_Position[1],self.Desired_Position[2] + 0.045]  
        self.Desired_Quaternion = [0.0, 1.0, 0.0, 0.0]


    def Gripper_control(self,Gripper,command):
    
        if command == 'open':
           Gripper.open()                  
        elif command == 'close':
            Gripper.close()
        else:
            print('The gripper command is not valid')

    def MoveRightArm(self):
        print "Moving Right Camera to watch the block"
        JointAngle1 = {'right_e0':-0.2017, 'right_e1':1.1562, 'right_s0':1.5056, 'right_s1':-0.4229, 
                       'right_w0':-2.8785, 'right_w1':-0.8893, 'right_w2':0.3973 }
        self.Arm_right.move_to_joint_positions(JointAngle1, timeout=15.0, threshold=0.008726646)

    def MoveAwayRightArm(self):
        print "Moving Right Camera away"
        JointAngle2 = {'right_e0':-0.2017, 'right_e1':1.1562, 'right_s0':0.8087, 'right_s1':-0.4229, 
                       'right_w0':-2.8785, 'right_w1':-0.8893, 'right_w2':0.3973 }
        self.Arm_right.move_to_joint_positions(JointAngle2, timeout=15.0, threshold=0.008726646)


    def MoveLeftArm_to_Block(self):
        JointAngle3 = {'left_e0':0.0360, 'left_e1':1.7974, 'left_s0':-1.0300, 'left_s1':-0.6481, 
                       'left_w0':-0.1050, 'left_w1':0.4111, 'left_w2':-0.1909 }
        self.Arm_left.move_to_joint_positions(JointAngle3, timeout=15.0, threshold=0.008726646)    

    def MoveLeftArm_to_House(self):
        JointAngle4 = {'left_e0':-0.1257, 'left_e1':1.5328, 'left_s0':-0.4916, 'left_s1':-0.7017, 
                       'left_w0':0.1027, 'left_w1':0.7098, 'left_w2':0.0429}
        self.Arm_left.move_to_joint_positions(JointAngle4, timeout=15.0, threshold=0.008726646)        


    def MainFunction(self):
        self.MoveRightArm()
        self.MoveLeftArm_to_House()
        raw_input('Everything is Ready. Press ''Enter'' to go!!!')

        ### While Loop ###
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Control rate timeout.")
                break
            self.Counting =  self.Counting + 1
            self.MoveRightArm() #Move right arm to get camera reading
            time.sleep(1) #Wait to allow good camera readings
            self.FindBlock()          # Use AR-Tag to get the position/orientation of next target
            self.MoveAwayRightArm()    # Move Right Away from workspace to Allow Left arm retrieval
            self.MoveBlock()          # Left arm apJointAngleproach to the target; gripper close; move to desired position; gripper open
            self.UpdateDesiredPosition()   # Update the desired position (expecially the height) for the next target


def main():
    # initialization
    BH = BuildHouse()
    # run main function
    BH.MainFunction()
    

if __name__ == "__main__":
    main()





## When finished shut down moveit_commander.
# moveit_commander.roscpp_shutdown()


