#!/usr/bin/env python
import rospy
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
# below are for the force control:
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint, 
    MotionWaypointOptions,
    InteractionOptions
)
import intera_interface


class LowLevelMotion(object):

    def __init__(self):
        self.jointNames = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        self.jointAngles = [0, 0, 0, 0, 0, 0, 0]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))
        self.positioning_pose = Pose()
        
    def joints_callback(self, data):
        # updating the actual arm joint angles by subscribing to the joint_states topic
        self.jointAngles = [data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6], data.position[7]]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))

    # move the robot based on joint command
    def basicPositionMove(self, pos, speed):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.joints_callback)
        rate = rospy.Rate(10)
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
        # terminate the control once the arm moved to the desired joint space with the threshold
        while control_diff_record>0.02:
            pub.publish(command)
            for x,y in zip(self.jointAngles, command.position):
                control_diff_temp = abs(x-y) + control_diff_temp
            control_diff_record = control_diff_temp
            control_diff_temp = 0.0
            rate.sleep()

        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")

    # this function is for moving the lower arm only to fulfill high frequency requirement for some massage patterns
    def lowerArmBasicMove(self, cur_pos, speed):
        # define some parameters:
        frequency = 100 # customize this to change the vibration speed
        repetation = 20

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

    def movetozero(self):
        rospy.init_node('move_to_zero', anonymous=True)
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        
        rate = rospy.Rate(10)

        pub_speed_ratio = rospy.Publisher('/robot/limb/right/set_speed_ratio', Float64, latch=True, queue_size=10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        command.mode = 1
        pub_speed_ratio.publish(0.2)

        start_time = rospy.get_time()
        end_time = rospy.get_time()
        rospy.loginfo(">>>>>>>>>> moving the arm >>>>>>>>>>>")
        while end_time-start_time<5:
            pub.publish(command)
            rate.sleep()
            end_time = rospy.get_time()


    def basicVelocityMove(self):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        rospy.init_node('lo_level_basic_ctrl', anonymous=True)
        rate = rospy.Rate(10)

        command = JointCommand()
        command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        command.velocity = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
        command.mode = 2
        
        pub.publish(command)

    def moveToPoint(self, position, speed):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = position[3]
        pose.orientation.y = position[4]
        pose.orientation.z = position[5]
        pose.orientation.w = position[6]
        self.ik(pose, speed)

    def ik(self, pose, speed):
        iksvc = rospy.ServiceProxy('ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append('right_hand')

        resp = iksvc(ikreq)
        if (resp.result_type[0] > 0):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo(">>>>>>>>>> moving the arm >>>>>>>>>>")
            self.basicPositionMove(limb_joints, speed)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

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

    # press down the end effector
    def press(self, waypoint, speed, press_depth):
        press_position = [waypoint[0], waypoint[1], waypoint[2]-press_depth, waypoint[3], waypoint[4], waypoint[5], waypoint[6]]
        retrive_position = waypoint
        self.moveToPoint(press_position, speed)
        rospy.sleep(0.5)
        self.moveToPoint(retrive_position, speed+0.1)

    def set_interaction_options(self, force):
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(True)
        interaction_options.set_interaction_control_mode([1, 1, 2, 1, 1, 1])
        interaction_options.set_in_endpoint_frame(True)
        # set the force:
        interaction_options.set_force_command([0.0, 0.0, force, 0.0, 0.0, 0.0])
        return interaction_options.to_msg()

    def exitForceControl(self):
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        interaction_options = InteractionOptions()
        interaction_options.set_interaction_control_active(False)
        msg = interaction_options.to_msg()
        pub.publish(msg)

    def forcePress(self, force, duration):
        rospy.loginfo(">>>>>>>>>> exerting force >>>>>>>>>>")
        force_msg = self.set_interaction_options(force)
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        rospy.logerr("publishiung")
        pub.publish(force_msg)
        rospy.sleep(duration)
        self.exitForceControl()

    def prePress(self, wayPoint):
        self.moveToPoint(wayPoint, 0.2)
        self.forcePress(40, 0)

    def lift(self, cur_waypoint, height):
        des_waypoint = [cur_waypoint[0], cur_waypoint[1], cur_waypoint[2]+height, cur_waypoint[3], cur_waypoint[4], cur_waypoint[5], cur_waypoint[6]]
        self.moveToPoint(des_waypoint, 0.2)
    
    def endpoint_callback(self, data):
        self.positioning_pose = data.pose

    def positioning_the_endpoint_to_force(self, force):
        rospy.loginfo(">>>>>>>>>> exerting force >>>>>>>>>>")
        force_msg = self.set_interaction_options(force)
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        pub.publish(force_msg)
        sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.endpoint_callback)
        rospy.sleep(1)
        self.exitForceControl()


    # below are some modularized massage patterns:
    def normalPress(self, wayPoints, lift_height, move_speed, press_duration):
        self.moveToPoint(wayPoints[0], move_speed)
        self.forcePress(40, press_duration)
        self.lift(wayPoint1, lift_height)

        self.moveToPoint(wayPoints[1], move_speed)
        self.forcePress(30, press_duration)

        self.moveToPoint(wayPoints[2], move_speed)
        self.forcePress(20, press_duration)

        self.moveToPoint(wayPoints[3], move_speed)
        self.forcePress(30, press_duration)
        self.lift(wayPoint4, move_speed)

    def rapidRepeatedPress(self, wayPoints, lift_height, move_speed, press_duration):
        press_wayPoints = []
        release_wayPoints = wayPoints 
        fast_speed = move_speed

        for wayPoint in wayPoints:
            press_wayPoint = [wayPoint[0], wayPoint[1], wayPoint[2]-0.03, wayPoint[3], wayPoint[4], wayPoint[5], wayPoint[6]]
            press_wayPoints.append(press_wayPoint)

        self.moveToPoint(wayPoints[0], move_speed)
        for i in range(0,3):
            self.moveToPoint(press_wayPoints[0], fast_speed)
            self.moveToPoint(release_wayPoints[0], fast_speed)

        self.moveToPoint(wayPoints[1], move_speed)
        for i in range(0,3):
            self.moveToPoint(press_wayPoints[1], fast_speed)
            self.moveToPoint(release_wayPoints[1], fast_speed)

        self.moveToPoint(wayPoints[2], move_speed)
        for i in range(0,3):
            self.moveToPoint(press_wayPoints[2], fast_speed)
            self.moveToPoint(release_wayPoints[2], fast_speed)

        self.moveToPoint(wayPoints[3], move_speed)
        for i in range(0,3):
            self.moveToPoint(press_wayPoints[3], fast_speed)
            self.moveToPoint(release_wayPoints[3], fast_speed)

    def armVibrate(self, wayPoints, lift_height, move_speed, press_duration):
        # start the high speed vibrating massage:
        counter = 0
        arm.moveToPoint(wayPoints[0], move_speed)
        arm.positioning_the_endpoint_to_force(10)
        for wayPoint in wayPoints:
            self.moveToPoint(wayPoint, move_speed)
            # move to the position specified by the desired force
            self.positioning_the_endpoint_to_force(10)
            # convert the waypoint to joint angles
            temp = self.waypointToJoint(self.positioning_pose)
            # vibrate the lower arm
            self.lowerArmBasicMove(temp, move_speed*2)
            # lift up the arm
            self.lift(wayPoints[counter], 0.05)
            counter = counter+1

if __name__ == '__main__':
    try:
        # define some way points:
        wayPoint1 = [0, 0.6, 0.05, 1, 0, 0, 0]
        wayPointTemp = [0, 0.7, 0.05, 1, 0, 0, 0]
        wayPoint2 = [0, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint3 = [-0.2, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint4 = [-0.2, 0.6, 0.05, 1, 0, 0, 0]
        wayPoints = [wayPoint1, wayPoint2, wayPoint3, wayPoint4]

        # define some pattern parameters:
        lift_height = 0.1
        move_speed = 0.2 # ratio to the max speed
        press_duration = 2 # seconds
        press_force = 20 # the vibrating pressing force

        # initialize a LowLevelMotion object
        arm = LowLevelMotion()
        # move the arm to zero position first
        arm.movetozero()
        # configure the force press issue
        #arm.prePress(wayPoint1)

        # start the massage pattern:
        #arm.normalPress(wayPoints, lift_height, move_speed, press_duration)

        # configure the force press issue
        #arm.prePress(wayPoint1)

        # start another massage pattern
        #arm.rapidRepeatedPress(wayPoints, lift_height-0.06, move_speed, press_duration-1)

        # configure the force press issue
        #arm.prePress(wayPoint1)
        # start another massage pattern
        arm.armVibrate(wayPoints, lift_height, move_speed, press_duration)
    except rospy.ROSInterruptException:
        pass
