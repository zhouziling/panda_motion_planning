#!/usr/bin/env python
 
 
"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""
 
 
import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
 
 
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
 
 
class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
                
        # Initialize the move group for the right arm
        right_arm = moveit_commander.MoveGroupCommander('panda_arm')
                
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set the reference frame for pose targets
        reference_frame = 'panda_link0'
        
        # Set the right arm reference frame accordingly
        right_arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.01)
        right_arm.set_goal_orientation_tolerance(0.05)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('resting')
 
 
        right_arm.go()
	
        #traj0 = right_arm.plan()
 
 
        # Execute the planned trajectory
 
 
        #right_arm.execute(traj0)
	rospy.loginfo("resting resting resting resting resting")
 
 
        rospy.sleep(1)
 
 
 
 
 
 
 
 
 
 
	joint_positions = [-1.0, -1.0, -1.0,-1.0, -1.0, -1.0]
 
        # Set the arm's goal configuration to the be the joint positions
        right_arm.set_joint_value_target(joint_positions)
        rospy.loginfo("0000  set_fk_target_completed  0000 ")        
        # Plan and execute the motion
        ######robot_arm.go()  
	traj = right_arm.plan()
        #rospy.loginfo("11111111111111111111111111 "+ str(traj))  
        # Execute the planned trajectory
        right_arm.execute(traj)
	
 
 
	current_pose = right_arm.get_current_pose(end_effector_link)
        #rospy.loginfo("1111   execute traj completed  1111   \n "+ str(current_pose))  
	rospy.loginfo("1111   execute traj completed  1111   \n ")
 
 
        rospy.sleep(1)
 
 
 
 
 
 
 
 
	# Get the end-effector pose
        ee_pose = right_arm.get_current_pose(end_effector_link)
        
        # Display the end-effector pose
        rospy.loginfo("End effector target pose:\n" + str(ee_pose))	
 
 
	#rospy.loginfo("eeeeeeeeeeeeeeeeeee:\n" + str(ee_pose.pose.position.x))
	
 
 
 
 
	# Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('forward')
 
 
        right_arm.go()
	
        #traj0 = right_arm.plan()
 
 
        # Execute the planned trajectory
 
 
        #right_arm.execute(traj0)
	rospy.loginfo("forward  forward  forward  forward  forward")
 
 
        rospy.sleep(1)
	
 
 
 
 
               
        # Set the target pose.  This particular pose has the gripper oriented horizontally
        # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
        # the center of the robot base.
 
 
		
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
 
 
		     
        target_pose.pose.position.x =ee_pose.pose.position.x
        target_pose.pose.position.y =ee_pose.pose.position.y
        target_pose.pose.position.z =ee_pose.pose.position.z
	target_pose.pose.orientation.x =ee_pose.pose.orientation.x
        target_pose.pose.orientation.y =ee_pose.pose.orientation.y
        target_pose.pose.orientation.z =ee_pose.pose.orientation.z
        target_pose.pose.orientation.w =ee_pose.pose.orientation.w
 
 
	
 
 
	'''
	
	target_pose.pose.position.x =0.18188
        target_pose.pose.position.y =0.04373
        target_pose.pose.position.z =0.12941
	target_pose.pose.orientation.x =0.8753672
        target_pose.pose.orientation.y =0.22988
        target_pose.pose.orientation.z =0.27960
        target_pose.pose.orientation.w =0.32048
'''
 
 
 
 
	rospy.loginfo("8888888888888888888"+ str(target_pose))    
   
        #Set the start state to the current state
	right_arm.set_start_state_to_current_state()
 
 
	       
        # Set the goal pose of the end effector to the stored pose
        right_arm.set_pose_target(target_pose, end_effector_link)
        
	
        ## Plan the trajectory to the goal
        traj1 = right_arm.plan()
        
	
        ## Execute the planned trajectory
        right_arm.execute(traj1)
 
 
	#rospy.loginfo("11111111111111111111111111"+ str(traj1))
	rospy.loginfo("111111  ik_executed successed   1111111")
    
        ## Pause for a second
        rospy.sleep(1)
 
 
 
 
	# Get the end-effector pose
        ee_pose_current = right_arm.get_current_pose(end_effector_link)
        
        # Display the end-effector pose
        rospy.loginfo("current ee_pose:\n" + str(ee_pose_current))	
 
 
 
 
 
 
	
        # Shift the end-effector to the right 5cm
        right_arm.shift_pose_target(2, 0.05, end_effector_link)
        ####right_arm.go()
 
 
        traj2 = right_arm.plan()
        
        #rospy.loginfo("22222222222222222222222222 "+ str(traj2))
	rospy.loginfo("222222   up    22222222")
	
        right_arm.execute(traj2)
 
 
        rospy.sleep(1)
 
 
        
	
 
        # Rotate the end-effector 90 degrees
        right_arm.shift_pose_target(5, 0.1, end_effector_link)
 
 
        #######right_arm.go()
 
 
	traj3 = right_arm.plan()
        
	#rospy.loginfo("333333333333333333333333333 "+ str(traj3))
	rospy.loginfo("333333   roll   3333333333 ")
 
 
        right_arm.execute(traj3)
 
 
        rospy.sleep(1)
 
 
	       
        # Store this pose as the new target_pose
        saved_target_pose = right_arm.get_current_pose(end_effector_link)
          
        # Move to the named pose "wave"
        right_arm.set_named_target('wave')
        #########right_arm.go()
 
 
	traj4 = right_arm.plan()
        
	#rospy.loginfo("4444444444444444444444444444444"+ str(traj4))
 
 
	rospy.loginfo("44444444  wave    444444444")
 
 
        right_arm.execute(traj4)
 
 
        rospy.sleep(1)
 
 
 
 
 
 
	  
        # Go back to the stored target
        right_arm.set_pose_target(saved_target_pose, end_effector_link)
        #########right_arm.go()
	traj5 = right_arm.plan()
        
	#rospy.loginfo("55555555555555555555555555"+ str(traj5))
	rospy.loginfo("5555555   saved_pose    55555555")
 
 
        right_arm.execute(traj5)
 
 
	
 
 
        rospy.sleep(1)
           
        # Finish up in the resting position  
        right_arm.set_named_target('resting')
        #########right_arm.go()
 
 
	traj6 = right_arm.plan()
        
	#rospy.loginfo("666666666666666666666666666"+ str(traj6))
	rospy.loginfo("6666666   resting    66666666")
 
 
        right_arm.execute(traj6)
 
 
 
 
        # Shut down MoveIt cleanly
	moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
	moveit_commander.os._exit(0)
 
 
 
 
 
 
if __name__ == "__main__":
    MoveItDemo()
 
