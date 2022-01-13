#!/usr/bin/env python
import time

from gripper.onrobot_vgc10 import OnRobotVGC10

import copy

import rospy
import roslaunch
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
import rospkg

import argparse
import os
import cv2
import json
import sys
import numpy as np
import open3d as o3d

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

from zivid_camera.srv import *

from termios import tcflush, TCIFLUSH

from agnostic_segmentation_live_demo import agnostic_segmentation

bridge = CvBridge()

rgb_img = None
depth_img = None

class Zivid:
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
    rospy.init_node('grasp_demo', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)

    rospack = rospkg.RosPack()
    rospack.list()

    while pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for iiwa robot connection")
        rospy.sleep(0.1)

    # setup zivid
    z = Zivid()

    # setup gripper
    gripper = OnRobotVGC10('172.28.60.249')
    gripper.set_current_limit(1000)

    position = list()
    orientation = list()
    # capture positions
    # bin
    position.append([0.19808839004579837, -0.19584869359690654, 0.8489844502089937])
    orientation.append([0.3573726487038741, 0.9105716943740845, -0.07019201209709704, 0.1954919107142917])
    # table
    position.append([0.2779583675156155, 0.26275243409152355, 0.7777829205816456])
    orientation.append([-0.35845471352424463, 0.9005717039108276, 0.09117319958905871, 0.2284038586206499])

    ROI = [np.array([[440,255],[1455,260],[1505,920],[400,930]]), np.array([[480,90],[1450,90],[1680,1045],[360,1090]])]
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'

    count = 0
    while True:
        i = 0
        # start cloud saver
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        rgb_sub = rospy.Subscriber('/zivid_camera/color/image_color', Image, rgb_callback, (i))
        depth_sub = rospy.Subscriber('/zivid_camera/depth/image', Image, depth_callback, (i))

        tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # format Pose msg
        msg.header.seq = count
        msg.pose.position.x = position[i][0]
        msg.pose.position.y = position[i][1]
        msg.pose.position.z = position[i][2]
        msg.pose.orientation.x = orientation[i][0]
        msg.pose.orientation.y = orientation[i][1]
        msg.pose.orientation.z = orientation[i][2]
        msg.pose.orientation.w = orientation[i][3]
        pub.publish(msg)
        rospy.sleep(10)  # wait till robot reach goal position
        z.capture_assistant_suggest_settings()  # TODO is capture assistant required after every frame?
        rospy.sleep(3)
        z.capture()
        pc2_msg = rospy.wait_for_message("/zivid_camera/points/xyzrgba", PointCloud2)
        rospy.sleep(1)  # wait camera capture frame and point cloud saver saves it

        rgb_sub.unregister()
        depth_sub.unregister()

        # background removal
        rgb_img_filtered = rgb_img
        #t = 0.005  # tolerance
        #bk = np.load(rospack.get_path('klt_dataset_collector') + "/agnostic_segmentation_live_demo/background_masks/bin_depth.npy")
        #rgb_img_filtered = copy.deepcopy(rgb_img)
        #d = depth_img
        #bk_mask = np.logical_and(bk - t <= d, d <= bk + t)
        #bk_mask = np.logical_and(bk_mask, ~np.isnan(bk))
        #print(np.where(bk_mask)[0].shape)
        #rgb_img_filtered[bk_mask] = np.array([0,0,0])
        #color = [255,255,255]
        #stencil = np.zeros(rgb_img.shape).astype(rgb_img.dtype)
        #cv2.fillPoly(stencil, [ROI[i]], color)
        #rgb_img_filtered = cv2.bitwise_and(rgb_img_filtered, stencil)
        #cv2.imshow('Image', rgb_img_filtered)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        # extract instances
        model_path = '/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/agnostic_segmentation_live_demo/agnostic_segmentation_model.pth'
        seg_img, predictions = agnostic_segmentation.segment_image(rgb_img, rgb_img, model_path)

        cv2.imshow('Segmented image', seg_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        instance = predictions["instances"].to("cpu")
        masks = np.array([instance.pred_masks[i].cpu().detach().numpy() for i in range(len(instance))])
        index = np.argmax([len(np.argwhere(x == True)) for x in masks]) # use the object with the biggest mask
        pred_masks = instance.pred_masks[index].cpu().detach().numpy()
        object_masked_rgb_img = rgb_img.copy()
        object_masked_rgb_img[pred_masks == False] = np.array([0, 0, 0])

        cv2.imshow('Image', object_masked_rgb_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # convert all instances to point cloud
        # TODO change it to loop all instances
        object_masked_depth_img = depth_img.copy()
        object_masked_depth_img = object_masked_depth_img.astype(float)
        x = np.argwhere(np.isnan(object_masked_depth_img))
        #object_masked_depth_img[np.argwhere(np.isnan(object_masked_depth_img))] = 0
        object_masked_depth_img[pred_masks == False] = 0
        object_masked_depth_img = np.float32(object_masked_depth_img)
        object_masked_depth_img = o3d.geometry.Image(object_masked_depth_img)
        object_masked_rgb_img = o3d.geometry.Image(object_masked_rgb_img)

        # segment a flat plane from the object
        intrinsic = o3d.camera.PinholeCameraIntrinsic(1944, 1200, 1778.81005859375, 1778.870361328125, 967.9315795898438, 572.4088134765625)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(object_masked_rgb_img, object_masked_depth_img, depth_scale=1, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        #o3d.visualization.draw_geometries([pcd])

        # extract surface from target object and calculate center
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.005, ransac_n=5, num_iterations=1000)
        plane_cloud = pcd.select_by_index(inliers)
        plane_cloud.paint_uniform_color([1.0, 0, 0])
        o3d.visualization.draw_geometries([plane_cloud])
        exit()

        # move gripper 20 cm above object center
        grasp_point = plane_cloud.get_center()
        grasp = copy.copy(msg)
        grasp.header.frame_id = 'zivid_optical_frame'
        #grasp.header.stamp = rospy.Time.now()
        grasp.pose.position.x = grasp_point[0]
        grasp.pose.position.y = grasp_point[1]
        grasp.pose.position.z = grasp_point[2] # 20 cm above grasp point
        # transform point to iiwa_link_0 before publishing it
        transform = tf_buffer.lookup_transform("iiwa_link_0",
                                               grasp.header.frame_id,
                                               rospy.Time(0),
                                               rospy.Duration(1.0))
        grasp_transformed = tf2_geometry_msgs.do_transform_pose(grasp, transform)
        # TODO later use plane center normal as the orientation
        grasp_transformed.pose.orientation.x = 0.422
        grasp_transformed.pose.orientation.y = 0.905
        grasp_transformed.pose.orientation.z = 0.014
        grasp_transformed.pose.orientation.w = 0.026
        #grasp_transformed.pose.orientation.x = orientation[i][0]
        #grasp_transformed.pose.orientation.y = orientation[i][1]
        #grasp_transformed.pose.orientation.z = orientation[i][2]
        #grasp_transformed.pose.orientation.w = orientation[i][3]
        print(grasp)
        print(grasp_transformed)
        # TODO subtract frame after reading it from TF
        grasp_transformed.pose.position.x = grasp_transformed.pose.position.x - 0.02
        grasp_transformed.pose.position.y = grasp_transformed.pose.position.y - 0.02
        grasp_transformed.pose.position.z = grasp_transformed.pose.position.z + 0.2 + 0.2
        #grasp_transformed.header.stamp = rospy.Time.now()
        pub.publish(grasp_transformed)
        rospy.sleep(10)  # wait till robot reach goal position

        # start suction move 20 down to grasp center
        gripper.set_pressure_channel_A(80)
        grasp_transformed.pose.position.z = grasp_transformed.pose.position.z - 0.2
        pub.publish(grasp_transformed)
        rospy.sleep(10)

        # go up to collect position
        grasp_transformed.pose.position.z = grasp_transformed.pose.position.z + 0.3
        pub.publish(grasp_transformed)
        rospy.sleep(10)

        # drop object to target
        gripper.release_channel_A()

        exit()

        count += 1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
