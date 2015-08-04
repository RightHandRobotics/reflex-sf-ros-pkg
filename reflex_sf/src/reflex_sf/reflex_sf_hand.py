#!/usr/bin/env python

from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from reflex_msgs.msg import ReflexCommand
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import TorqueCommand
from reflex_msgs.msg import VelocityCommand
import reflex_msgs.msg
import motor


class ReflexSFHand():
    def __init__(self):
        self.namespace = '/reflex_sf'
        rospy.init_node('reflex_sf')
        rospy.loginfo('Starting up the ReFlex SF hand')
        self.motors = {self.namespace + '_f1': motor.Motor(self.namespace + '_f1'),
                       self.namespace + '_f2': motor.Motor(self.namespace + '_f2'),
                       self.namespace + '_f3': motor.Motor(self.namespace + '_f3'),
                       self.namespace + '_preshape': motor.Motor(self.namespace + '_preshape')}
        rospy.Subscriber(self.namespace + '/command', ReflexCommand, self.receive_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_position', PoseCommand, self.receive_angle_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_velocity', VelocityCommand, self.receive_vel_cmd_cb)
        rospy.Subscriber(self.namespace + '/command_motor_torque', TorqueCommand, self.receive_torque_cmd_cb)
        self.hand_state_pub = rospy.Publisher(self.namespace + '/hand_state', reflex_msgs.msg.Hand, queue_size=10)
        rospy.Service(self.namespace + '/zero_fingers', Empty, self.calibrate)
        rospy.loginfo('ReFlex SF hand has started, waiting for commands...')

    def receive_cmd_cb(self, data):
        self.disable_torque_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)

    def receive_angle_cmd_cb(self, data):
        self.disable_torque_control()
        self.reset_speeds()
        self.set_angles(data)

    def receive_vel_cmd_cb(self, data):
        self.disable_torque_control()
        self.set_velocities(data)

    def receive_torque_cmd_cb(self, data):
        self.reset_speeds()
        self.set_torque_cmds(data)
        self.enable_torque_control()

    def set_angles(self, pose):
        self.motors[self.namespace + '_f1'].set_motor_angle(pose.f1)
        self.motors[self.namespace + '_f2'].set_motor_angle(pose.f2)
        self.motors[self.namespace + '_f3'].set_motor_angle(pose.f3)
        self.motors[self.namespace + '_preshape'].set_motor_angle(pose.preshape)

    def set_velocities(self, velocity):
        self.motors[self.namespace + '_f1'].set_motor_velocity(velocity.f1)
        self.motors[self.namespace + '_f2'].set_motor_velocity(velocity.f2)
        self.motors[self.namespace + '_f3'].set_motor_velocity(velocity.f3)
        self.motors[self.namespace + '_preshape'].set_motor_velocity(velocity.preshape)

    def set_speeds(self, speed):
        self.motors[self.namespace + '_f1'].set_motor_speed(speed.f1)
        self.motors[self.namespace + '_f2'].set_motor_speed(speed.f2)
        self.motors[self.namespace + '_f3'].set_motor_speed(speed.f3)
        self.motors[self.namespace + '_preshape'].set_motor_speed(speed.preshape)

    def set_torque_cmds(self, torque):
        self.motors[self.namespace + '_f1'].set_torque_cmd(torque.f1)
        self.motors[self.namespace + '_f2'].set_torque_cmd(torque.f2)
        self.motors[self.namespace + '_f3'].set_torque_cmd(torque.f3)
        self.motors[self.namespace + '_preshape'].set_torque_cmd(torque.preshape)

    def reset_speeds(self):
        for ID, motor in self.motors.items():
            motor.reset_motor_speed()

    def disable_torque_control(self):
        for ID, motor in self.motors.items():
            motor.disable_torque_control()

    def enable_torque_control(self):
        for ID, motor in self.motors.items():
            motor.enable_torque_control()

    def disable_torque(self):
        for ID, motor in self.motors.items():
            motor.disable_torque()

    def enable_torque(self):
        for ID, motor in self.motors.items():
            motor.enable_torque()

    def publish_hand_state(self):
        state = reflex_msgs.msg.Hand()
        motor_names = ('_f1', '_f2', '_f3', '_preshape')
        for i in range(4):
            state.motor[i] = self.motors[self.namespace + motor_names[i]].get_motor_msg()
        self.hand_state_pub.publish(state)

    def calibrate(self, data=None):
        for motor in sorted(self.motors):
            rospy.loginfo("Calibrating motor " + motor)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")
            while not command.lower() == 'q':
                if command.lower() == 't' or command.lower() == 'tt':
                    print "Tightening motor " + motor
                    self.motors[motor].tighten(0.35 * len(command) - 0.3)
                elif command.lower() == 'l' or command.lower() == 'll':
                    print "Loosening motor " + motor
                    self.motors[motor].loosen(0.35 * len(command) - 0.3)
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            rospy.loginfo("Saving current position for %s as the zero point", motor)
            self.motors[motor].set_local_motor_zero_point()
        print "Calibration complete, writing data to file"
        self.zero_current_pose()
        return []

    def write_zero_point_data_to_file(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path("reflex_sf")
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def zero_current_pose(self):
        data = dict(
            reflex_sf_f1=dict(zero_point=self.motors[self.namespace + '_f1'].get_current_raw_motor_angle()),
            reflex_sf_f2=dict(zero_point=self.motors[self.namespace + '_f2'].get_current_raw_motor_angle()),
            reflex_sf_f3=dict(zero_point=self.motors[self.namespace + '_f3'].get_current_raw_motor_angle()),
            reflex_sf_preshape=dict(zero_point=self.motors[self.namespace + '_preshape'].get_current_raw_motor_angle())
        )
        self.write_zero_point_data_to_file('reflex_sf_zero_points.yaml', data)


def main():
    rospy.sleep(5.0)  # To allow parameters to load
    hand = ReflexSFHand()
    rospy.on_shutdown(hand.disable_torque)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        hand.publish_hand_state()
        r.sleep()


if __name__ == '__main__':
    main()
