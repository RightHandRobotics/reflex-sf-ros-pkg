import rospy

from reflex_sf_msgs.msg import SFPose
from reflex_sf_hand import ReflexSFHand

def main():
    rospy.init_node("finger_test")
    pub = rospy.Publisher('/reflex_sf/command', SFPose, queue_size=2)
    cmd = SFPose()
    count = 0

    while not rospy.is_shutdown():
        rospy.loginfo("Opening hand")
        cmd.f2 = 0.0
        pub.publish(cmd)
        pub.publish(cmd)
        rospy.sleep(1.0)
        rospy.loginfo("Closing hand, count %d", count)
        cmd.f2 = 1.5
        pub.publish(cmd)
        pub.publish(cmd)
        rospy.sleep(0.75)
        count += 1

if __name__ == '__main__':
    main()
