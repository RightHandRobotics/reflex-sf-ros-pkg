import rospy
 
from reflex_msgs.msg import PoseCommand
 
rospy.init_node('reflex_sf_dof_tour')
cmd_publisher = rospy.Publisher('/reflex_sf/command_position', PoseCommand, queue_size=10)

FINGER_CLOSED = 4.6
FINGER_PINCH = 3.5 

PRESHAPE_CYLINDER = 0
PRESHAPE_SPHERICAL = 1.5
PRESHAPE_PINCH = 2.5

# finger 1 (right-hand index) close
rospy.sleep(5)
cmd_publisher.publish(0,0,0,0)
rospy.sleep(1)
cmd_publisher.publish(FINGER_CLOSED, 0, 0, 0)
rospy.sleep(1)
cmd_publisher.publish(0, 0, 0, 0)
rospy.sleep(1)
# finger 2 (right-hand middle) close
cmd_publisher.publish(0, FINGER_CLOSED, 0, 0)
rospy.sleep(1)
cmd_publisher.publish(0, 0, 0, 0)
rospy.sleep(1)
# finger 3 (thumb) close
cmd_publisher.publish(0, 0, FINGER_CLOSED, 0)
rospy.sleep(1)
cmd_publisher.publish(0, 0, 0, 0)
rospy.sleep(1)
# preshape
cmd_publisher.publish(0, 0, 0, PRESHAPE_SPHERICAL)
rospy.sleep(1)
cmd_publisher.publish(0, 0, 0, PRESHAPE_PINCH)
rospy.sleep(1)
cmd_publisher.publish(0, 0, 0, 0)
rospy.sleep(1)
# hand closed in cylindrical power grasp
cmd_publisher.publish(FINGER_CLOSED, FINGER_CLOSED, FINGER_CLOSED, 0)
rospy.sleep(1)
# hand open
cmd_publisher.publish(0, 0, 0, 0)
rospy.sleep(1)
# preshape hand for pinch
cmd_publisher.publish(0, 0, 0, PRESHAPE_PINCH)
rospy.sleep(1)
# pinch grasp
cmd_publisher.publish(FINGER_PINCH, FINGER_PINCH, 0, PRESHAPE_PINCH)
rospy.sleep(1)
# hand open (pinch grasp)
cmd_publisher.publish(0, 0, 0, PRESHAPE_PINCH)
rospy.sleep(1)
# hand open (cylindrical grasp)
cmd_publisher.publish(0, 0, 0, 0)
rospy.sleep(1)
