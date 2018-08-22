#! /usr/bin/env python

import move_to_point
import move_limb_to_neutral
import math
import PyKDL
import rospy
import serial
import struct

from std_msgs.msg import String
from kitronyx.msg import Force

from intera_core_msgs.msg import InteractionControlCommand
import argparse
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface.utility_functions import int2bool
import random
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf_conversions import posemath
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint, 
    MotionWaypointOptions,
    InteractionOptions
)
import intera_interface
from intera_interface import(
    CHECK_VERSION,
    Limb
)

class CircularMotion():

    def __init__(self):
        self.mover = move_to_point.MoveToPoint()
        self.retrieve_height = -0.015
        self._limb = Limb()
        self.traj_options = TrajectoryOptions()
        self.traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        self.traj = MotionTrajectory(trajectory_options = self.traj_options, limb = self._limb)
        self.wpt_options = MotionWaypointOptions(max_linear_speed = 0.4,
                                        max_linear_accel = 0.4,
                                        max_rotational_speed = 1.57,
                                        max_rotational_accel = 1.57,
                                        max_joint_speed_ratio = 1.0)
        self.waypoint = MotionWaypoint(options=self.wpt_options.to_msg(), limb=self._limb)
        self.joint_names = self._limb.joint_names()
        endpoint_state = self._limb.tip_state('right_hand')
        self.pose = endpoint_state.pose


    def to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(0.5*yaw)
        sy = math.sin(0.5*yaw)
        cr = math.cos(0.5*roll)
        sr = math.sin(0.5*roll)
        cp = math.cos(0.5*pitch)
        sp = math.sin(0.5*pitch)

        qx = cy*sr*cp - sy*cr*sp
        qy = cy*cr*sp + sy*sr*cp
        qz = sy*cr*cp - cy*sr*sp
        qw = cy*cr*cp + sy*sr*sp

        orientation = [qx, qy, qz, qw]
        return orientation


    def displacement(self, time):
        disp = 0.1667*math.pi*math.sin(time)
        return disp

    def move(self, cur_pos):
        curr_roll = 1.1667*math.pi
        curr_pitch = 0
        curr_yaw = 0
        self.mover.move(cur_pos[0], cur_pos[1], self.retrieve_height, cur_pos[3], cur_pos[4], cur_pos[5], cur_pos[6])
        rospy.sleep(1)
        time = 0

        while time<2*math.pi:

            end = self.to_quaternion(curr_roll, curr_pitch, curr_yaw)
            self.pose.position.x = cur_pos[0]
            self.pose.position.y = cur_pos[1]
            self.pose.position.z = self.retrieve_height
            self.pose.orientation.x = end[0]
            self.pose.orientation.y = end[1]
            self.pose.orientation.z = end[2]
            self.pose.orientation.w = end[3]

            if time<0.5*math.pi:
                curr_roll = 1.1667*math.pi - self.displacement(time) 
            elif time<math.pi:
                curr_roll = 0.833*math.pi + self.displacement(time) 
            elif time<1.5*math.pi:
                curr_roll = 0.833*math.pi - self.displacement(time)
            else:
                curr_roll = 1.1667*math.pi + self.displacement(time)
                
            curr_pitch = self.displacement(time)
            time = time+0.1

            poseStamped = PoseStamped()
            poseStamped.pose = self.pose
            self.waypoint.set_cartesian_pose(poseStamped, 'right_hand', [])
            self.traj.append_waypoint(self.waypoint.to_msg())

        result = self.traj.send_trajectory(timeout=None)