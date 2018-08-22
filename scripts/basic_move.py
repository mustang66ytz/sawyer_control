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
        
    def callback(self, data):
        # updating the actual arm joint angles by subscribing to the joint_states topic
        self.jointAngles = [data.position[1], data.position[2], data.position[3], data.position[4], data.position[5], data.position[6], data.position[7]]
        self.actualPos = dict(zip(self.jointNames, self.jointAngles))

    # move the robot based on joint command
    def basicPositionMove(self, pos, speed):
        pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
        sub = rospy.Subscriber('/robot/joint_states', JointState, self.callback)
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

        rospy.sleep(1)
        rospy.loginfo(">>>>>>>>>> The robot is at the target position <<<<<<<<<<:")
        rospy.loginfo(pos['right_j0'])
        rospy.loginfo(pos['right_j1'])
        rospy.loginfo(pos['right_j2'])
        rospy.loginfo(pos['right_j3'])
        rospy.loginfo(pos['right_j4'])
        rospy.loginfo(pos['right_j5'])
        rospy.loginfo(pos['right_j6'])

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

    # press down the end effector
    def press(self, waypoint, speed, press_depth):
        press_position = [waypoint[0], waypoint[1], waypoint[2]-press_depth, waypoint[3], waypoint[4], waypoint[5], waypoint[6]]
        retrive_position = waypoint
        self.moveToPoint(press_position, speed)
        rospy.sleep(0.5)
        self.moveToPoint(retrive_position, speed+0.1)

    def set_interaction_options(self, force):
        n_dim_cart = 6
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

    def forcePress(self, force):
        rospy.loginfo(">>>>>>>>>> exerting force >>>>>>>>>>")
        force_msg = self.set_interaction_options(force)
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=1)
        rospy.logerr("publishiung")
        pub.publish(force_msg)
        rospy.sleep(2)
        self.exitForceControl()

if __name__ == '__main__':
    try:
        # define some way points:
        wayPoint1 = [0, 0.6, 0.05, 1, 0, 0, 0]
        wayPoint2 = [0, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint3 = [-0.2, 0.8, 0.05, 1, 0, 0, 0]
        wayPoint4 = [-0.2, 0.6, 0.05, 1, 0, 0, 0]
        # initialize a LowLevelMotion object
        arm = LowLevelMotion()
        # move the arm to zero position first
        arm.movetozero()
        arm.moveToPoint(wayPoint1, 0.2)
        arm.moveToPoint(wayPoint1, 0.2)
        arm.forcePress(40)
        arm.moveToPoint(wayPoint2, 0.2)
        arm.forcePress(30)
        arm.moveToPoint(wayPoint3, 0.1)
        arm.forcePress(20)
        arm.moveToPoint(wayPoint4, 0.2)
        
    except rospy.ROSInterruptException:
        pass
