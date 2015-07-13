import rospy

from reflex_sf_msgs.msg import SFPose
from reflex_sf_hand import ReflexSFHand

def main():
    rospy.init_node("finger_test")
    pub = rospy.Publisher('/reflex_sf/command', SFPose, queue_size=2)
    cmd = SFPose()
    rospy.sleep(1.0)
    pub.publish(cmd); pub.publish(cmd)
    pub.publish(cmd); pub.publish(cmd)
    rospy.sleep(5.0)
    cmd.f2 = 4.5
    cmd.f3 = 4.0
    count = 0

    while not rospy.is_shutdown():
        rospy.loginfo("Opening hand")
        cmd.f1 = 0.0
        pub.publish(cmd); pub.publish(cmd)
        rospy.sleep(2.0)
        rospy.loginfo("Closing hand, count %d", count)
        cmd.f1 = 4.0
        pub.publish(cmd); pub.publish(cmd)
        rospy.sleep(2.0)
        count += 1

if __name__ == '__main__':
    main()
