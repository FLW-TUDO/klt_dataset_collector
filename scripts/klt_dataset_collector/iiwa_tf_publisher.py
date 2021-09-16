#!/usr/bin/env python
import rospy
import tf
from iiwa_msgs.msg import CartesianPose

br = tf.TransformBroadcaster()

def callback(data):
    
    br.sendTransform((data.poseStamped.pose.position.x,data.poseStamped.pose.position.y,data.poseStamped.pose.position.z),
                     (data.poseStamped.pose.orientation.x,data.poseStamped.pose.orientation.y,data.poseStamped.pose.orientation.z,data.poseStamped.pose.orientation.w),
                     rospy.Time.now(),
                     "iiwa_link_ee",
                     "iiwa_link_0")
    
    br.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"iiwa_link_0","world")
    br.sendTransform((0.86, 0, -0.575),(0,0,0,1),rospy.Time.now(),"bin_link","world")
    br.sendTransform((1.285, 0, -0.46),(0,0,0,1),rospy.Time.now(),"table_link","world")

def main():
    '''
    This code is temporary it should be abandoned when software on iiwa KRC is updated. It should publish the TF tree then.
    '''
    rospy.init_node('iiwa_tf_broadcaster')
    rospy.Subscriber("iiwa/state/CartesianPose", CartesianPose, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
