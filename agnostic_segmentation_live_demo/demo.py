#!/usr/bin/env python
import copy

import rospy
import roslaunch
import tf
from cv_bridge import CvBridge, CvBridgeError
import rospkg

import argparse
import os
import cv2
import json
import sys
import numpy as np

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

from zivid_camera.srv import *

from termios import tcflush, TCIFLUSH

from agnostic_segmentation_live_demo import agnostic_segmentation

bridge = CvBridge()

rgb_img = None
depth_img = None

class Sample:
    def __init__(self):
        rospy.loginfo("Starting sample_capture_assistant.py")

        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service, 29.0)
        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

        rospy.Subscriber("/zivid_camera/points/xyzrgba", PointCloud2, self.on_points)

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(10)
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.1f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time,
            ambient_light_frequency=CaptureAssistantSuggestSettingsRequest.AMBIENT_LIGHT_FREQUENCY_NONE,
        )

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_service()

    def on_points(self, data):
        rospy.loginfo("PointCloud received")


def rgb_callback(msg, args):
    global rgb_img
    rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def depth_callback(msg, args):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #cv2.imwrite("/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/agnostic_segmentation_live_demo/background_masks/bin_depth.png", depth_img)
    #np.save("/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/agnostic_segmentation_live_demo/background_masks/table_depth.npy", depth_img)

def main():
    global rgb_img, depth_img
    rospy.init_node('data_collector', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)

    listener = tf.TransformListener()

    rospack = rospkg.RosPack()
    rospack.list()

    while pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for iiwa robot connection")
        rospy.sleep(0.1)

    # setup zivid
    s = Sample()

    position = list()
    orientation = list()
    # capture positions
    # bin
    position.append([0.40010042658960016, -0.40230712128976354, 0.586433233868703])
    orientation.append([0.38184669676554683, 0.9182227253913879, -0.040387323075866455, 0.0971030723192087])
    # table
    position.append([0.2779583675156155, 0.26275243409152355, 0.7777829205816456])
    orientation.append([-0.35845471352424463, 0.9005717039108276, 0.09117319958905871, 0.2284038586206499])

    ROI = [np.array([[440,255],[1455,260],[1505,920],[400,930]]), np.array([[480,90],[1450,90],[1680,1045],[360,1090]])]
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'

    count = 0
    while True:
        #os.system('clear')
        #tcflush(sys.stdin, TCIFLUSH)
        #input("Press Enter to start capturing")

        for i in range(len(position)):
            # start cloud saver
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            rgb_sub = rospy.Subscriber('/zivid_camera/color/image_color', Image, rgb_callback, (i))
            depth_sub = rospy.Subscriber('/zivid_camera/depth/image', Image, depth_callback, (i))

            # format Pose msg
            msg.header.seq = i
            msg.pose.position.x = position[i][0]
            msg.pose.position.y = position[i][1]
            msg.pose.position.z = position[i][2]
            msg.pose.orientation.x = orientation[i][0]
            msg.pose.orientation.y = orientation[i][1]
            msg.pose.orientation.z = orientation[i][2]
            msg.pose.orientation.w = orientation[i][3]
            pub.publish(msg)
            rospy.sleep(10)  # wait till robot reach goal position
            s.capture_assistant_suggest_settings()  # TODO is capture assistant required after every frame?
            rospy.sleep(3)
            s.capture()
            pc2_msg = rospy.wait_for_message("/zivid_camera/points/xyzrgba", PointCloud2)
            rospy.sleep(1)  # wait camera capture frame and point cloud saver saves it

            rgb_sub.unregister()
            depth_sub.unregister()

            # background removal
            t = 0.003  # tolerance
            if i == 0:  # bin
                bk = np.load(rospack.get_path('klt_dataset_collector') + "/agnostic_segmentation_live_demo/background_masks/bin_depth.npy")
            elif i == 1:  # table
                bk = np.load(rospack.get_path('klt_dataset_collector') + "/agnostic_segmentation_live_demo/background_masks/table_depth.npy")
            rgb_img_filtered = copy.deepcopy(rgb_img)
            d = depth_img
            bk_mask = np.logical_and(bk - t <= d, d <= bk + t)
            bk_mask = np.logical_and(bk_mask, ~np.isnan(bk))
            print(np.where(bk_mask)[0].shape)
            rgb_img_filtered[bk_mask] = np.array([0,0,0])
            color = [255,255,255]
            stencil = np.zeros(rgb_img.shape).astype(rgb_img.dtype)
            cv2.fillPoly(stencil, [ROI[i]], color)
            rgb_img_filtered = cv2.bitwise_and(rgb_img_filtered, stencil)
            cv2.imshow('Image', rgb_img_filtered)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            seg_img = agnostic_segmentation.segment_image(rgb_img_filtered, rgb_img)
            cv2.imshow('Image', seg_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            rospy.sleep(1)

        count += 1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
