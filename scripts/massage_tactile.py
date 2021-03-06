#!/usr/bin/env python
import rospy
import numpy as np
import math
import serial
import PyKDL
from scipy import interpolate
import matplotlib.pyplot as plt
from std_msgs.msg import (
    String,
    Float64,
    Header
)
from kitronyx.msg import Force
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
# below are for the force control:
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint, 
    MotionWaypointOptions,
    InteractionOptions
)
import intera_interface


class LowLevelMotionWithTactileSensor(object):

    def __init__(self):
        # force related:
        self.curr_force = 0
        self.force_threshold = 10
        self.correct_depth = 0
        # limb related:
        self.jointNames = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6'] # define the arm's joints names
        self.jointAngles = [0, 0, 0, 0, 0, 0, 0]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles)) # Store the actual joint positions of the arm
        self.positioning_pose = Pose() # Store the position of the end effector under specific force
    
    # called by the joint state subscriber
    def joints_callback(self, data):
        # updating the actual arm joint angles by subscribing to the joint_states topic
        self.jointAngles = [data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6], data.position[7]]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))

    # move the robot based on joint command
    def basicPositionMove(self, pos, speed):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        rate = rospy.Rate(180)
        # define the speed publisher
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [pos['right_j0'], pos['right_j1'], pos['right_j2'], pos['right_j3'], pos['right_j4'], pos['right_j5'], pos['right_j6']]
        command.mode = 1
        # customize the value to change speed
        pub_speed_ratio.publish(speed)
        
        control_diff_record = 10.0
        control_diff_temp = 0.0
        threshold = 0.02
        # terminate the control once the arm moved to the desired joint space within the threshold
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while control_diff_record>threshold:
            end_time = rospy.get_time()
            if end_time-start_time>5:
                break
            pub.publish(command)
            for x,y in zip(self.jointAngles, command.position):
                control_diff_temp = abs(x-y) + control_diff_temp
            control_diff_record = control_diff_temp
            control_diff_temp = 0.0
            rate.sleep()

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")
    
    # move the robot based on joint command
    def basicPositionMoveForPositioning(self, pos, speed):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        rate = rospy.Rate(180)
        # define the speed publisher
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [pos['right_j0'], pos['right_j1'], pos['right_j2'], pos['right_j3'], pos['right_j4'], pos['right_j5'], pos['right_j6']]
        command.mode = 1
        # customize the value to change speed
        pub_speed_ratio.publish(speed)
        # terminate the control once the arm moved to the desired joint space within the threshold
        
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
        # terminate the control once the arm moved to the desired joint space within the threshold
        counter = 0
        while counter<traj_length-1:
            command.position = [positions[counter]['right_j0'], positions[counter]['right_j1'], positions[counter]['right_j2'], positions[counter]['right_j3'], positions[counter]['right_j4'], positions[counter]['right_j5'], positions[counter]['right_j6']]
            pub.publish(command)
            #for x,y in zip(self.jointAngles, command.position):
                #control_diff_temp = abs(x-y) + control_diff_temp
            #control_diff_record = control_diff_temp
            #control_diff_temp = 0.0
            rate.sleep()
            counter = counter+1

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")

    # this function is for moving the lower arm only to fulfill high frequency requirement for some massage patterns
    def lowerArmBasicMove(self, cur_pos, speed):
        # define some parameters:
        frequency = 100 # customize this to change the vibration speed
        repetation = 10

        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        rate = rospy.Rate(frequency)
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)
        # set the joint angles for the first press position
        command1 = JointCommand()
        command1.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command1.position = [cur_pos['right_j0'], cur_pos['right_j1'], cur_pos['right_j2'], cur_pos['right_j3']-0.05, cur_pos['right_j4']-0.05, cur_pos['right_j5']+0.1, cur_pos['right_j6']]
        command1.mode = 1
        # set the joint angles for the second press position
        command2 = JointCommand()
        command2.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command2.position = [cur_pos['right_j0'], cur_pos['right_j1'], cur_pos['right_j2'], cur_pos['right_j3'], cur_pos['right_j4'], cur_pos['right_j5'], cur_pos['right_j6']]
        command2.mode = 1
        # looping to create vibrating motion
        pub_speed_ratio.publish(speed)
        for i in range(0, repetation):
            for j in range(0, 10):
                pub.publish(command1)
                rate.sleep()
            rate.sleep()
            for k in range(0, 10):
                pub.publish(command2)
                rate.sleep()
            rate.sleep()

    # move the arm to zero joint position
    def movetozero(self):
        rospy.init_node('arm_low_level_control_with_tactile_sensor', anonymous=True)
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        command.mode = 1
        pub_speed_ratio.publish(0.2)

        start_time = rospy.get_time()
        end_time = rospy.get_time()
        rospy.loginfo(">>>>>>>>>> moving the arm to zero joint position >>>>>>>>>>>")
        while end_time-start_time<8:
            pub.publish(command)
            end_time = rospy.get_time()
    
    # move the arm to the specified position with the specifies speed
    def moveToPoint(self, position, speed, positioning):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = position[3]
        pose.orientation.y = position[4]
        pose.orientation.z = position[5]
        pose.orientation.w = position[6]
        # call the inverse kinematics function
        self.ik(pose, speed, positioning)

    # inverse kinematics function integrating inverse kinematics service and send the joint angles to basicPositionMove function to publish joint commands
    def ik(self, pose, speed, positioning):
        iksvc = rospy.ServiceProxy('ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')

        resp = iksvc(ikreq)
        # execute the joint command if the inverse kinematics is successful
        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo(">>>>>>>>>> moving the arm >>>>>>>>>>")
            if positioning:
                self.basicPositionMoveForPositioning(limb_joints, speed)
            else:
                self.basicPositionMove(limb_joints, speed)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

    # call the ikservice to convert way point to joint space
    def waypointToJoint(self, pose):
        iksvc = rospy.ServiceProxy('ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')

        resp = iksvc(ikreq)
        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

    def forceCallback(self, data):
        sum = 0
        for i in range(160):
            sum = sum+data.current[i]
        rospy.loginfo("total force current: %d", sum)
        self.curr_force = sum

    # tactile sensor listener
    def listener(self):
        rospy.Subscriber('tactile', Force, self.forceCallback)
    
    # positioning the end effector to reach the desired force
    def findDepthFromForce(self, cur_pos, ref_force, speed):
        cur_depth = cur_pos[2]
        depth_step = 0.0005
        while self.curr_force+self.force_threshold<ref_force:
            desired_pos = [cur_pos[0], cur_pos[1], cur_depth-depth_step, cur_pos[3], cur_pos[4], cur_pos[5], cur_pos[6]]
            self.moveToPoint(desired_pos, speed, True)
            cur_depth = cur_depth-depth_step
        self.correct_depth = cur_depth
        rospy.logwarn("reached the desired force")
        
    # lift the robot arm after massaging a point
    def lift(self, cur_waypoint, height):
        des_waypoint = [cur_waypoint[0], cur_waypoint[1], cur_waypoint[2]+height, cur_waypoint[3], cur_waypoint[4], cur_waypoint[5], cur_waypoint[6]]
        self.moveToPoint(des_waypoint, 0.2, False)
    
    # convert roll pitch yaw to quaternion
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
    
    def endpoint_callback(self, data):
        self.positioning_pose = data.pose

    # positioning the end effector according to the desired force
    def positioning_the_endpoint_to_force(self, pos, force, speed):
        rospy.loginfo(">>>>>>>>>> exerting force >>>>>>>>>>")
        self.findDepthFromForce(pos, force, speed)
        sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_callback)
        rospy.sleep(0.5)

    def moveACircle(self, wayPoint, move_speed):
        # define some key way points of the circle
        speed_rate = 20
        rolls_start_position = 7*math.pi/6
        rolls_end_position = 5*math.pi/6
        pitch_start_position = 0
        pitch_end_position = 0
        number_of_interpolation = 30
        curr_yaw = 0
        # the first half circle
        rolls = [rolls_start_position, math.pi, rolls_end_position]
        pitches = [pitch_start_position, math.pi/6, pitch_end_position]

        f = interpolate.interp1d(rolls, pitches, kind='quadratic')
        rolls_new = np.linspace(rolls_start_position, rolls_end_position, number_of_interpolation)
        i = 0
        circle_points_joints = []
        
        for i in range (0,len(rolls_new)-1):
            end = self.to_quaternion(rolls_new[i], f(rolls_new)[i], curr_yaw)
            pose = Pose()
            pose.position.x = self.positioning_pose.position.x
            pose.position.y = self.positioning_pose.position.y
            pose.position.z = self.positioning_pose.position.z
            pose.orientation.x = end[0]
            pose.orientation.y = end[1]
            pose.orientation.z = end[2]
            pose.orientation.w = end[3]
            circle_points_joints.append(self.waypointToJoint(pose))
        self.basicTrajMove(circle_points_joints, move_speed, len(rolls_new), speed_rate)

        # the second half circle
        rolls = [rolls_end_position, math.pi, rolls_start_position]
        pitches = [pitch_end_position, -math.pi/6, pitch_start_position]
        f = interpolate.interp1d(rolls, pitches, kind='quadratic')
        rolls_new = np.linspace(rolls_end_position, rolls_start_position, number_of_interpolation)
        i = 0
        circle_points_joints = []
        
        for i in range (0,len(rolls_new)-1):
            end = self.to_quaternion(rolls_new[i], f(rolls_new)[i], curr_yaw)
            pose = Pose()
            pose.position.x = self.positioning_pose.position.x
            pose.position.y = self.positioning_pose.position.y
            pose.position.z = self.positioning_pose.position.z
            pose.orientation.x = end[0]
            pose.orientation.y = end[1]
            pose.orientation.z = end[2]
            pose.orientation.w = end[3]
            circle_points_joints.append(self.waypointToJoint(pose))
        self.basicTrajMove(circle_points_joints, move_speed, len(rolls_new), speed_rate)


    def moveACircleCalibrated(self, wayPoint, move_speed, tool_length):
        # define some key way points of the circle
        speed_rate = 150
        circle_radius = tool_length*math.sin(math.pi/6)
        x_start_position = self.positioning_pose.position.x
        x_end_position = self.positioning_pose.position.x
        y_start_position = self.positioning_pose.position.y - circle_radius
        y_end_position = self.positioning_pose.position.y + circle_radius
        rolls_start_position = 7*math.pi/6
        rolls_end_position = 5*math.pi/6
        pitch_start_position = 0
        pitch_end_position = 0
        number_of_interpolation = 400
        curr_yaw = 0

        # the first half circle
        x_positions = [x_start_position, x_start_position+circle_radius, x_end_position]
        y_positions = [y_start_position, y_start_position+circle_radius, y_end_position]
        rolls = [rolls_start_position, math.pi, rolls_end_position]
        pitches = [pitch_start_position, math.pi/6, pitch_end_position]

        f = interpolate.interp1d(rolls, pitches, kind='quadratic')
        g = interpolate.interp1d(y_positions, x_positions, kind = 'quadratic')
        y_positions_new = np.linspace(y_start_position, y_end_position, number_of_interpolation)
        rolls_new = np.linspace(rolls_start_position, rolls_end_position, number_of_interpolation)
        i = 0
        circle_points_joints = []
        
        for i in range (0,len(rolls_new)-1):
            end = self.to_quaternion(rolls_new[i], f(rolls_new)[i], curr_yaw)
            pose = Pose()
            pose.position.x = g(y_positions_new)[i]
            pose.position.y = y_positions_new[i]
            pose.position.z = self.positioning_pose.position.z+0.1*circle_radius
            pose.orientation.x = end[0]
            pose.orientation.y = end[1]
            pose.orientation.z = end[2]
            pose.orientation.w = end[3]
            circle_points_joints.append(self.waypointToJoint(pose))
        self.basicTrajMove(circle_points_joints, move_speed, len(rolls_new), speed_rate)

        # the second half circle
        
        x_positions = [x_end_position, x_end_position-circle_radius, x_start_position]
        y_positions = [y_end_position, y_end_position-circle_radius, y_start_position]
        rolls = [rolls_end_position, math.pi, rolls_start_position]
        pitches = [pitch_end_position, -math.pi/6, pitch_start_position]
        f = interpolate.interp1d(rolls, pitches, kind='quadratic')
        g = interpolate.interp1d(y_positions, x_positions, kind = 'quadratic')
        y_positions_new = np.linspace(y_end_position, y_start_position, number_of_interpolation)
        rolls_new = np.linspace(rolls_end_position, rolls_start_position, number_of_interpolation)
        i = 0
        circle_points_joints = []
        
        for i in range (0,len(rolls_new)-1):
            end = self.to_quaternion(rolls_new[i], f(rolls_new)[i], curr_yaw)
            pose = Pose()
            pose.position.x = g(y_positions_new)[i]
            pose.position.y = y_positions_new[i]
            pose.position.z = self.positioning_pose.position.z+0.1*circle_radius
            pose.orientation.x = end[0]
            pose.orientation.y = end[1]
            pose.orientation.z = end[2]
            pose.orientation.w = end[3]
            circle_points_joints.append(self.waypointToJoint(pose))
        self.basicTrajMove(circle_points_joints, move_speed, len(rolls_new), speed_rate)

    # below are some modularized massage patterns:
    # normal press pattern with the waypoints, move_speed, and force specified
    def normalPress(self, wayPoints, lift_height, move_speed):
        force = [30, 30, 30, 30]

        for i in range(4):
            self.moveToPoint(wayPoints[i], move_speed, False)
            self.findDepthFromForce(wayPoints[i], force[i], move_speed-0.1)
            rospy.sleep(1)
            self.lift(wayPoints[i], lift_height)
    
    # repeated normal press
    def repeatedPress(self, wayPoints, lift_height, move_speed, repetition):
        force = [30, 30, 30, 30]
        for i in range(4):
            for j in range(5):
                self.moveToPoint(wayPoints[i], move_speed, False)
                self.findDepthFromForce(wayPoints[i], force[i], move_speed-0.1)
                rospy.sleep(1)
                self.lift(wayPoints[i], lift_height)

    # high-frequency, high-speed lower arm padding pattern
    def armVibrate(self, wayPoints, lift_height, move_speed):
        # start the high speed vibrating massage:
        counter = 0
        for wayPoint in wayPoints:
            self.moveToPoint(wayPoint, move_speed, False)
            # move to the position specified by the desired force
            self.positioning_the_endpoint_to_force(wayPoint, 30, move_speed-0.1)
            # convert the waypoint to joint angles
            temp = self.waypointToJoint(self.positioning_pose)
            # vibrate the lower arm
            self.lowerArmBasicMove(temp, move_speed)
            # lift up the arm
            self.lift(wayPoints[counter], 0.05)
            counter = counter+1
    
    def circularMotion(self, wayPoints, lift_height, move_speed, calibrated):
        counter = 0
        repetition = 2
        for wayPoint in wayPoints:
            self.moveToPoint(wayPoint, move_speed, False)
            # move to the position specified by the desired force
            self.positioning_the_endpoint_to_force(wayPoint, 25, move_speed-0.1)
            # move a circle
            if calibrated:
                tool_length = 0.05
                for i in range(0,repetition):
                    self.moveACircleCalibrated(self.positioning_pose, move_speed, tool_length)
                    if not i == repetition-1:
                        self.lift(wayPoints[counter], 0.01)
                        self.positioning_the_endpoint_to_force(wayPoint, 25, move_speed-0.1)
                    
            if not calibrated:
                self.moveACircle(self.positioning_pose, move_speed+0.05)
            # lift up the arm
            self.lift(wayPoints[counter], 0.05)
            counter = counter+1

if __name__ == '__main__':
    try:
        # define some way points:
        wayPoint1 = [0, 0.6, 0.05, 1, 0, 0, 0]
        wayPointTemp = [0, 0.7, 0.0, 1, 0, 0, 0]
        wayPoint2 = [0, 0.65, 0.05, 1, 0, 0, 0]
        wayPoint3 = [-0.2, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint4 = [-0.2, 0.6, 0.05, 1, 0, 0, 0]
        wayPoints = [wayPoint1, wayPoint2, wayPoint3, wayPoint4]

        # define some pattern parameters:
        lift_height = 0.1
        move_speed = 0.25 # ratio to the max speed
        press_duration = 2 # seconds
        press_force = 20 # the vibrating pressing force

        # initialize a LowLevelMotionWithTactileSensor object
        arm = LowLevelMotionWithTactileSensor()
        arm.listener()
        # move the arm to zero position first
        arm.movetozero()

        # start the massage pattern:
        arm.normalPress(wayPoints, lift_height, move_speed)
        arm.repeatedPress(wayPoints, 0, move_speed, 5)
        arm.armVibrate(wayPoints, lift_height, move_speed)
        #arm.circularMotion(wayPoints, lift_height, move_speed, True)
    except rospy.ROSInterruptException:
        pass