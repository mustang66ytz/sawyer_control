#!/usr/bin/env python

import argparse
import operator
import sys
import threading

from bisect import bisect
from copy import copy
from os import path

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from std_msgs.msg import (
    Float64,
    Header
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import (
    JointCommand,
    EndpointState
)
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

class Trajectory(object):
    def __init__(self):
        self.jointNames = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'] # define the arm's joints names
        self.jointAngles = [0, 0, 0, 0, 0, 0, 0] 
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))
        self.init_point = False

    # called by the joint state subscriber
    def joints_callback(self, data):
        # updating the actual arm joint angles by subscribing to the joint_states topic
        self.jointAngles = [data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6], data.position[7]]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))

    def basicPositionMove(self, pos, speed, initial):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        # define the speed publisher
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]]
        command.mode = 1
        # customize the value to change speed
        pub_speed_ratio.publish(speed)
        # terminate the control once the arm moved to the desired joint space within the threshold

        rate = rospy.Rate(1000)
        control_diff_record = 10.0
        control_diff_temp = 0.0
        threshold = 0.02
        if not initial:
        # terminate the control once the arm moved to the desired joint space within the threshold
            for i in range(0, 200):
                pub.publish(command)
                rate.sleep()
        else:
            for i in range(0, 10000):
                rospy.logerr("returning to the first point")
                pub.publish(command)
                rate.sleep()

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")

    def basicTrajMove(self, positions, speed, traj_length, speed_rate):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        rate = rospy.Rate(speed_rate)
        # define the speed publisher
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.mode = 1
        # customize the value to change speed
        pub_speed_ratio.publish(speed)
        
        control_diff_record = 10.0
        control_diff_temp = 0.0
        threshold = 0.015

        # terminate the control once the arm moved to the desired joint space within the threshold
        counter = 0
        while control_diff_record>threshold and counter<traj_length-1:
            command.position = [positions[counter][0], positions[counter][1], positions[counter][2], positions[counter][3], positions[counter][4], positions[counter][5], positions[counter][6]]
            pub.publish(command)
            for x,y in zip(self.jointAngles, command.position):
                control_diff_temp = abs(x-y) + control_diff_temp
            control_diff_record = control_diff_temp
            control_diff_temp = 0.0
            rate.sleep()
            counter = counter+1

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")

    def _clean_line(self, line, joint_names):
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        if line.find("special")==-1:
            line = [try_float(x) for x in line.rstrip().split(',')]
        else:
            line = [try_float(x) for x in line.strip("special").split(',')]
            rospy.logerr("At a special point")
            rospy.sleep(2)
        #convert it to a dictionary with only valid commands
        command = line[1:]
        return command
        
    def parse_file(self, filename):
        #rate = rospy.Rate(100)
        # open the file:
        with open(filename, 'r') as f:
            lines = f.readlines()
        trajectory = []

        for idx, values in enumerate(lines[1:]):
            print(idx)
            goal_pose = self._clean_line(values, self.jointNames)
            if idx == 0:
                self.init_point = True
            else:
                self.init_point = False
            trajectory.append(goal_pose)
            #self.basicTrajMove(trajectory, 0.2, len(trajectory), 20)
            self.basicPositionMove(goal_pose, 0.1, self.init_point)
            #rate.sleep()

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        '-n', '--number_loops', type=int, default=1,
        help='number of playback loops. 0=infinite.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("taozheng_joint_trajectory_file_playback")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = Trajectory()
    traj.parse_file(path.expanduser(args.file))

if __name__ == "__main__":
    main()