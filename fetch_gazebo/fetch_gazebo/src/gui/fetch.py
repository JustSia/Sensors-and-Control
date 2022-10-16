#!/usr/bin/env python3

from ast import Pass
import sys

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandGoal,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math

import threading

import roslib

roslib.load_manifest("teleop_twist_keyboard")
import rospy

from geometry_msgs.msg import Twist

from sensor_msgs.msg import JointState

# from gui_widgets.moveit import Grasp


class Robot:
    """
    Class the handle fetch robot ROS messages and joint positions

    Program Flow:
    Subscribe to fetch robot topics
    create lists of joint names and positions to used in the class
    create joint limits for teleop commands
    create links to all controllers needed for moving the fetch robot
    
    Methods:
    - arm_reset, pre-grasp pose, pickObj -> hard coded position values for arm to travel too.
    - update commands for GUI teleop mode and methods listed above
    - execute commands send the ROS messages to move the robot include arm, head, base and bellows
    """
    def __init__(self, robot_name):
        # self.grasping = Grasp()
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        #
        # Arm Joints Info
        #
        # Joint Names
        self.arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "upperarm_roll_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
        ]
        # Joint Inital Positions
        self.joint_positions = [0.0, -0.62, 0, 0, 0.0, 0.62, 0.0]

        # Saved Joint Positions
        self.joint_zero_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_pre_grasp_position = [1.57, 0.3, 0.0, -2.0, 0.0, -0.57, 0.0]

        self.joint_bin_position = [-1.23, 0.75, -1.67, 0.75, 0.36, -2.04, 1.04]

        self.joint_pre_grab = [-0.612, 0.32, -1.076, -2.064, 0.0495, 1.3777, 0.98]


        self.jointMax = [
            math.pi / 2,
            math.pi / 2,
            math.pi / 2,
            math.pi / 2,
            math.pi,
            math.pi / 2,
            math.pi / 2,
        ]
        self.jointMin = [
            -math.pi / 2,
            -math.pi / 2,
            -math.pi / 2,
            -math.pi / 2,
            -math.pi,
            -math.pi / 2,
            -math.pi / 2,
        ]

        #
        # Head Joints Info
        #
        # Head Names
        self.head_joint_names = ["head_pan_joint", "head_tilt_joint"]
        # Head Inital Positions
        self.head_joint_positions = [0.0, 0.0]

        self.gripper_joint_names = ["l_gripper_finger_joint"]

        #
        # Gripper Info
        #
        # Gripper Inital Position
        self.gripper_position = 0

        rospy.Subscriber("/joint_states", JointState, self.jointCallback)

        self.currentJointStates = [0] * 12

        rospy.loginfo("Waiting for head_controller...")
        self.head_client = actionlib.SimpleActionClient(
            "head_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        self.head_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient(
            "arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction
        )
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for torso_controller...")
        self.torso_client = actionlib.SimpleActionClient(
            "torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        self.torso_client.wait_for_server()
        rospy.loginfo("...connected.")

        self.torso_joint_names = ["torso_lift_joint"]
        self.torso_joint_positions = [0]

        self.reset_arm()
        self.update_head(50)

    def preGrasp(self):
        self.joint_positions = self.joint_pre_grab
        self.execute_arm_update()        
    def binObj(self):
        current_states = list(self.joint_positions)
        current_states[1] = -0.25
        self.joint_positions = tuple(current_states)
        self.execute_arm_update()
        current_states = list(self.joint_positions)
        current_states[0] = -1.35
        self.joint_positions = tuple(current_states)
        self.execute_arm_update()
        self.update_gripper(100)
        self.joint_positions = self.joint_pre_grab
        self.execute_arm_update()

    def pick_obj(self):
        current_states = list(self.joint_positions)
        save = current_states[1]
        current_states[1] = -0.3
        self.joint_positions = tuple(current_states)
        self.execute_arm_update()
        self.update_torso(20)
        current_states[1] = save
        self.joint_positions = tuple(current_states)
        self.execute_arm_update()
        self.update_gripper(100)

    def grasp_seq(self):
        current_states = list(self.joint_positions)
        save = current_states[1]
        current_states[1] = -0.35
        self.joint_positions = tuple(current_states)
        self.execute_arm_update()
        current_states[1] = save
        self.joint_positions = tuple(current_states)
        self.execute_arm_update()

    def jointCallback(self, data):
        self.currentJointStates = data.position

    def reset_arm(self):
        self.joint_positions = self.joint_pre_grasp_position
        self.execute_arm_update()

    def update_arm_joints(self, arm_joint_num, pos):
        # map slider pos from 0 -> 9 to joint min and max
        percent = pos / 100.0

        m = 1 / (self.jointMax[arm_joint_num] - self.jointMin[arm_joint_num])
        b = -m * self.jointMin[arm_joint_num]
        value = (percent - b) / m

        self.joint_positions[arm_joint_num] = value
        self.execute_arm_update()

    def update_head(self, pos):
        # map 0 -> 99 to required range
        self.head_joint_positions[1] = pos / 100
        self.execute_head_update()

    def update_torso(self, pos):
        self.torso_joint_positions[0] = pos / 100 * 0.4
        self.execute_torso_update()

    def update_gripper(self, pos):
        # map 0 -> 99 to required range
        self.gripper_position = pos / 100 * 0.09
        self.execute_gripper_update()

    def getArmPose(self):
        return self.joint_positions

    def updateJoints(self,positions):
        self.joint_positions = positions

    def execute_arm_update(self):

        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.joint_positions
        trajectory.points[0].velocities = [0.0] * len(self.joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(4.0)

        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory = trajectory
        arm_goal.goal_time_tolerance = rospy.Duration(10.0)

        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result(rospy.Duration(15.0))

    def execute_head_update(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.head_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.head_joint_positions
        trajectory.points[0].velocities = [0.0] * len(self.head_joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.head_joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(0.1)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result(rospy.Duration(5.0))

    def execute_torso_update(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.torso_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.torso_joint_positions
        trajectory.points[0].velocities = [0.0] * len(self.torso_joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.torso_joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(0.1)

        print(self.torso_joint_positions[0])
        torso_goal = FollowJointTrajectoryGoal()
        torso_goal.trajectory = trajectory
        torso_goal.goal_time_tolerance = rospy.Duration(0.0)

        self.torso_client.send_goal(torso_goal)
        self.torso_client.wait_for_result(rospy.Duration(5.0))

    def execute_gripper_update(self):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 10.0
        gripper_goal.command.position = self.gripper_position
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(5.0))

    def execute_twist(self, x, y):

        twist = Twist()
        twist.linear.x = x
        twist.angular.z = y
        self.twist_pub.publish(twist)


    def grasp(self,x,y,z):
        pass