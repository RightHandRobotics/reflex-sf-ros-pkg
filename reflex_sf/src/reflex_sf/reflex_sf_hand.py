#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with a ReFlex SF hand


from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64

from reflex_sf_msgs.msg import SFPose
from reflex_sf_msgs.msg import SFVelocity
from motor import Motor


class ReflexSFHand(object):
    def __init__(self):
        rospy.init_node('reflex_sf')
        rospy.loginfo('Starting up the ReFlex SF hand')
        self.motors = {'/reflex_sf_f1': Motor('/reflex_sf_f1'),
                       '/reflex_sf_f2': Motor('/reflex_sf_f2'),
                       '/reflex_sf_f3': Motor('/reflex_sf_f3'),
                       '/reflex_sf_preshape': Motor('/reflex_sf_preshape')}
        rospy.Subscriber('/reflex_sf/position_command', SFPose, self.receivePosCmdCb)
        rospy.Subscriber('/reflex_sf/velocity_command', SFVelocity, self.receiveVelCmdCb)
        rospy.loginfo('ReFlex SF hand has started, waiting for commands...')

    def receivePosCmdCb(self, data):
        self.motors['/reflex_sf_f1'].setMotorPosition(data.f1)
        self.motors['/reflex_sf_f2'].setMotorPosition(data.f2)
        self.motors['/reflex_sf_f3'].setMotorPosition(data.f3)
        self.motors['/reflex_sf_preshape'].setMotorPosition(data.preshape)

    def receiveVelCmdCb(self, data):
        self.motors['/reflex_sf_f1'].setMotorVelocity(data.f1)
        self.motors['/reflex_sf_f2'].setMotorVelocity(data.f2)
        self.motors['/reflex_sf_f3'].setMotorVelocity(data.f3)

    def printMotorPositions(self):
        print ""  # visually separates following print messages in the flow
        for motor in sorted(self.motors):
            print motor, " position: ", self.motors[motor].getCurrentPosition()

    def calibrate(self):
        for motor in sorted(self.motors):
            rospy.loginfo("Calibrating motor " + motor)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")
            while not command.lower() == 'q':
                if command.lower() == 't' or command.lower() == 'tt':
                    print "Tightening motor " + motor
                    self.motors[motor].tighten(0.25 * len(command) - 0.2)
                elif command.lower() == 'l' or command.lower() == 'll':
                    print "Loosening motor " + motor
                    self.motors[motor].loosen(0.25 * len(command) - 0.2)
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            rospy.loginfo("Saving current position for %s as the zero point",
                          motor)
            self.motors[motor].setMotorZeroPoint()
        print "Calibration complete, writing data to file"
        self.writeCurrentPositionsToZero()

    def writeZeroPointDataToFile(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path("reflex_sf")
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def writeCurrentPositionsToZero(self):
        data = dict(
            reflex_sf_f1=dict(zero_point=self.motors['/reflex_sf_f1'].getRawCurrentPosition()),
            reflex_sf_f2=dict(zero_point=self.motors['/reflex_sf_f2'].getRawCurrentPosition()),
            reflex_sf_f3=dict(zero_point=self.motors['/reflex_sf_f3'].getRawCurrentPosition()),
            reflex_sf_preshape=dict(zero_point=self.motors['/reflex_sf_preshape'].getRawCurrentPosition())
        )
        self.writeZeroPointDataToFile('reflex_sf_zero_points.yaml', data)

    def disableTorque(self):
        for motor in self.motors:
            self.motors[motor].disableTorque()

    def enableTorque(self):
        for motor in self.motors:
            self.motors[motor].enableTorque()


def main():
    hand = ReflexSFHand()
    rospy.on_shutdown(hand.disableTorque)
    rospy.spin()


if __name__ == '__main__':
    main()
