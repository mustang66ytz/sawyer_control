#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

import rospy

import intera_interface

from intera_interface import CHECK_VERSION
from std_msgs.msg import String
import sys, select, termios, tty

class JointRecorder(object):
    def __init__(self, filename, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self.gripper_name = '_'.join([side, 'gripper'])
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_right = intera_interface.Limb(side)
        try:
            self._gripper = intera_interface.Gripper(self.gripper_name)
            rospy.loginfo("Electric gripper detected.")
        except Exception as e:
            self._gripper = None
            rospy.loginfo("No electric gripper detected.")

        # Verify Gripper Have No Errors and are Calibrated
        if self._gripper:
            if self._gripper.has_error():
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()

        self._cuff = intera_interface.Cuff(side)

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True
    

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == " ":
                return True
            return False
        else:
            return False

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        settings = termios.tcgetattr(sys.stdin)
        if self._filename:
            joints_right = self._limb_right.joint_names()
            with open(self._filename, 'w') as f:
                f.write('time,')
                temp_str = '' if self._gripper else '\n'
                f.write(','.join([j for j in joints_right]) + ',' + temp_str)
                if self._gripper:
                    f.write(self.gripper_name+'\n')
                while not self.done():
                    if self._gripper:
                        if self._cuff.upper_button():
                            self._gripper.open()
                        elif self._cuff.lower_button():
                            self._gripper.close()
                    angles_right = [self._limb_right.joint_angle(j)
                                    for j in joints_right]

                    if self.getKey():  # self added
                        f.write("special")

                    f.write("%f," % (self._time_stamp(),))
                    f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
                    if self._gripper:
                        f.write(str(self._gripper.get_position()) + '\n')
                    self._rate.sleep()
    




def main():
    """SDK Joint Recorder Example

    Record timestamped joint and gripper positions to a file for
    later play back.

    Run this example while moving the robot's arm and gripper
    to record a time series of joint and gripper positions to a
    new csv file with the provided *filename*. This example can
    be run in parallel with any other example or standalone
    (moving the arms in zero-g mode while pressing the cuff
    buttons to open/close grippers).

    You can later play the movements back using one of the
    *_file_playback examples.
    """
    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=1000, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_recorder")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    recorder = JointRecorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)


    print("Recording. Press Ctrl-C to stop.")
    recorder.record()

    print("\nDone.")

if __name__ == '__main__':
    main()
