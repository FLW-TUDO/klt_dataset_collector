#!/usr/bin/env python
import rospy
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    # calibration ma
    #cal_mat = np.array(
    #    [[ 0.015231,  0.999728, -0.017669, -85.517654],
    #     [-0.998412,  0.014248, -0.054505,  61.033829],
    #     [-0.054238,  0.018471,  0.998357,  57.805283],
    #     [ 0.000000,  0.000000,  0.000000,  1.000000 ]])
    #cal_mat = np.array(
    #    [[ 6.66428776e-03, 9.99196768e-01, -3.95152643e-02, -6.06342659e+01],
    #     [-9.98673916e-01, 4.63311747e-03, -5.12727089e-02,  4.38823738e+01],
    #     [-5.10484464e-02, 3.98045592e-02,  9.97902632e-01,  7.50844955e+01],
    #     [0.,              0.,              0.,              1.            ]])
    #cal_mat = np.array(
    #    [[-0.003881,  0.999948,  0.009453, -89.118034],
    #     [-0.998355, -0.003333, -0.057239,  62.759880],
    #     [-0.057205, -0.009660,  0.998316,  63.055576],
    #     [0.000000,   0.000000,  0.000000,  1.000000 ]])
    #cal_mat = np.array(
    #    [[0.002431, 0.999974, -0.006853, -92.570206],
    #     [-0.998457, 0.002047, -0.055500, 59.145885],
    #     [-0.055485, 0.006978, 0.998435, 61.066181],
    #     [0.000000, 0.000000, 0.000000, 1.000000]]
    #)
    #cal_mat = np.array(
    #    [[0.007679, 0.999880, -0.013486, -90.654182],
    #     [-0.998076, 0.006834, -0.061619, 69.445969],
    #     [-0.061520, 0.013934, 0.998009, 63.149094],
    #     [0.000000, 0.000000, 0.000000, 1.000000]]
    #)
    cal_mat = np.array(
        [[0.005284, 0.999897, -0.013365, -90.514549],
         [-0.998225, 0.004482, -0.059384, 68.331665],
         [-0.059318, 0.013655, 0.998146, 62.200356],
         [0.000000, 0.000000, 0.000000, 1.000000]]
    )
    r = R.from_matrix(cal_mat[0:3,0:3])
    translation = cal_mat[0:3, 3]/1000
    rotation = r.as_quat()
    print(translation)
    print(rotation)

    rospy.init_node('iiwa_zivid_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
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