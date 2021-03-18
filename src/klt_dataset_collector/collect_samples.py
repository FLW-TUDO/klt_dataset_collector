#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('data_collector', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    rate = rospy.Rate(0.1)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'

    msg.header.seq = 1
    msg.pose.position.x = 0.57
    msg.pose.position.y = -0.41
    msg.pose.position.z = 0.56
    msg.pose.orientation.x = 0.32
    msg.pose.orientation.y = 0.90
    msg.pose.orientation.z = -0.25
    msg.pose.orientation.w = -0.00

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    rospy.loginfo("Move to point")
    pub.publish(msg)
    rospy.sleep(5)

    rospy.loginfo("Move to point")
    pub.publish(msg)
    rospy.sleep(5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass