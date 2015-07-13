#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with a ReFlex SF hand


from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64

from reflex_sf_msgs.msg import SFCommand
from reflex_sf_msgs.msg import SFPose
from reflex_sf_msgs.msg import SFVelocity
import motor


class ReflexSFHand(object):
    def __init__(self):
        rospy.init_node('reflex_sf')
        rospy.loginfo('Starting up the ReFlex SF hand')
        self.motors = {'/reflex_sf_f1': motor.Motor('/reflex_sf_f1'),
                       '/reflex_sf_f2': motor.Motor('/reflex_sf_f2'),
                       '/reflex_sf_f3': motor.Motor('/reflex_sf_f3'),
                       '/reflex_sf_preshape': motor.Motor('/reflex_sf_preshape')}
        rospy.Subscriber('/reflex_sf/command', SFCommand, self.receive_cmd_cb)
        rospy.Subscriber('/reflex_sf/command_position', SFPose, self.receive_angle_cmd_cb)
        rospy.Subscriber('/reflex_sf/command_velocity', SFVelocity, self.receive_vel_cmd_cb)
        rospy.loginfo('ReFlex SF hand has started, waiting for commands...')

    def receive_cmd_cb(self, data):
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)

    def receive_angle_cmd_cb(self, data):
        self.reset_speeds()
        self.set_angles(data)

    def receive_vel_cmd_cb(self, data):
        self.set_velocities(data)

    def set_angles(self, pose):
        self.motors['/reflex_sf_f1'].set_motor_angle(pose.f1)
        self.motors['/reflex_sf_f2'].set_motor_angle(pose.f2)
        self.motors['/reflex_sf_f3'].set_motor_angle(pose.f3)
        self.motors['/reflex_sf_preshape'].set_motor_angle(pose.preshape)

    def set_velocities(self, velocity):
        self.motors['/reflex_sf_f1'].set_motor_velocity(velocity.f1)
        self.motors['/reflex_sf_f2'].set_motor_velocity(velocity.f2)
        self.motors['/reflex_sf_f3'].set_motor_velocity(velocity.f3)
        self.motors['/reflex_sf_preshape'].set_motor_velocity(velocity.preshape)

    def set_speeds(self, speed):
        self.motors['/reflex_sf_f1'].set_motor_speed(speed.f1)
        self.motors['/reflex_sf_f2'].set_motor_speed(speed.f2)
        self.motors['/reflex_sf_f3'].set_motor_speed(speed.f3)
        self.motors['/reflex_sf_preshape'].set_motor_speed(speed.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def disable_torque(self):
        for ID, motor in self.motors.items():
            motor.disable_torque()

    def enable_torque(self):
        for ID, motor in self.motors.items():
            motor.enable_torque()

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
            self.motors[motor].set_local_motor_zero_point()
        print "Calibration complete, writing data to file"
        self.write_current_angles_to_zero()

    def write_zero_point_data_to_file(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path("reflex_sf")
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def write_current_angles_to_zero(self):
        data = dict(
            reflex_sf_f1=dict(zero_point=self.motors['/reflex_sf_f1'].get_current_raw_angle()),
            reflex_sf_f2=dict(zero_point=self.motors['/reflex_sf_f2'].get_current_raw_angle()),
            reflex_sf_f3=dict(zero_point=self.motors['/reflex_sf_f3'].get_current_raw_angle()),
            reflex_sf_preshape=dict(zero_point=self.motors['/reflex_sf_preshape'].get_current_raw_angle())
        )
        self.write_zero_point_data_to_file('reflex_sf_zero_points.yaml', data)


def main():
    hand = ReflexSFHand()
    rospy.on_shutdown(hand.disable_torque)
    rospy.spin()


if __name__ == '__main__':
    main()
