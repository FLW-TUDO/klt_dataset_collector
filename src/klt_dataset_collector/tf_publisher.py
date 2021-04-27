#!/usr/bin/env python
import rospy
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    # calibration ma
    cal_mat = np.array(
      [ [ 0.015231,  0.999728, -0.017669, -85.517654], 
        [-0.998412,  0.014248, -0.054505,  61.033829], 
        [-0.054238,  0.018471,  0.998357,  57.805283], 
        [ 0.000000,  0.000000,  0.000000,  1.000000 ]])
    r = R.from_matrix(cal_mat[0:3,0:3])
    translation = cal_mat[0:3, 3]
    rotation = r.as_quat()
    print(translation)
    print(rotation)

    rospy.init_node('iiwa_zivid_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(tuple(translation),
                         tuple(rotation),
                         rospy.Time.now(),
                         "zivid_optical_frame",
                         "iiwa_link_ee")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass