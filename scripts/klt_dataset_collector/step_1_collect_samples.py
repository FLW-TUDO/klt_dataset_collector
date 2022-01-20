#!/usr/bin/env python
import rospy
import roslaunch
import tf
from cv_bridge import CvBridge, CvBridgeError

import argparse
import os
import cv2
import json
import sys

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

from zivid_camera.srv import *

from termios import tcflush, TCIFLUSH

bridge = CvBridge()

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
    rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(args[0] + '/rgb/' + f"{args[1]:06}" + '.png', rgb_img)

def depth_callback(msg, args):
    depth_img = bridge.imgmsg_to_cv2(msg, "passthrough")
    depth_img = depth_img * 10000  # convert resolution from meter to 1/10 mm
    depth_img = depth_img.astype('uint16')
    # Note depth scale in BOP is 10
    cv2.imwrite(args[0] + '/depth/' + f"{args[1]:06}" + '.png', depth_img)

def main():
    # read arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", default='/home/iiwa/segmentation/flw_dataset')
    parser.add_argument("--start-count", type=int, default=0)
    parser.add_argument("--filter-type", default='table', help='bin or table')
    args = parser.parse_args()

    if not os.path.exists(args.dataset_path):
        rospy.logerr("Error: dataset folder doesn't exist")
        exit()

    rospy.init_node('data_collector', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)

    listener = tf.TransformListener()

    if args.filter_type == 'bin':
        scene_link = 'bin_link'
    elif args.filter_type == 'table':
        scene_link = 'table_link'
    # setup PCD cloud saver
    cloud_saver_cli_args = ['input:=' + args.filter_type + '_filter/' + args.filter_type + '_filter/box_filtered',
                            '_binary:=True',
                            '_fixed_frame:=' + scene_link,
                            '_prefix:=']

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # setup zivid
    s = Sample()

    position = list()
    orientation = list()
    # capture positions
    if args.filter_type == 'bin':
        # middle
        position.append([0.387, 0, 0.663])
        orientation.append([0, 0.979, 0, 0.2])

        # more far1
        position.append([0.387, 0, 0.866])
        orientation.append([0, 0.979, 0, 0.2])
        # more far2
        position.append([0.387, 0, 0.947])
        orientation.append([0, 0.979, 0, 0.2])
        # more near1
        position.append([0.387, 0, 0.562])
        orientation.append([0, 0.979, 0, 0.2])
        # more near2
        position.append([0.387, 0, 0.472])
        orientation.append([0, 0.979, 0, 0.2])
        # top left
        position.append([0.387, 0, 0.663])
        orientation.append([-0.639, 0.716, -0.231, 0.158])
        # top right
        position.append([0.387, 0, 0.663])
        orientation.append([0.685, 0.699, 0.164, 0.122])
        # top back
        position.append([0.410, 0, 0.675])
        orientation.append([0.955, 0.076, 0.286, 0.023])
        # top center1
        position.append([0.492, 0, 0.660])
        orientation.append([0, 0.984, 0, 0.176])
        # top center3
        #position.append([0.688, 0, 0.650])
        #orientation.append([0.004, 0.995, 0, 0.089])
        # top center2
        position.append([0.684, 0, 0.652])
        orientation.append([0, 0.999, 0, 0.041])

        # left side1
        position.append([0.387, 0.070, 0.662])
        orientation.append([0, 0.979, 0, 0.20])
        # left side2
        position.append([0.397, 0.131, 0.669])
        orientation.append([-0.202, 0.947, -0.121, 0.215])
        # left side3
        position.append([0.501, 0.152, 0.496])
        orientation.append([-0.193, 0.949, -0.179, 0.169])
        # left side4
        position.append([0.594, 0.231, 0.494])
        orientation.append([-0.296, 0.913, -0.250, 0.125])
        # left side5
        position.append([0.747, 0.191, 0.450])
        orientation.append([-0.437, 0.863, -0.250, 0.037])

        # right side1
        position.append([0.387, -0.182, 0.663])
        orientation.append([0, 0.979, 0, 0.20])
        # right side2
        position.append([0.377, -0.164, 0.644])
        orientation.append([0.058, 0.977, 0.120, 0.16])
        # right side3
        position.append([0.437, -0.265, 0.595])
        orientation.append([0.248, 0.953, 0.120, 0.12])
        # right side4
        position.append([0.514, -0.248, 0.470])
        orientation.append([0.454, 0.857, 0.227, 0.086])
        # right side5
        position.append([0.552, -0.293, 0.424])
        orientation.append([0.739, 0.592, 0.320, -0.007])


    elif args.filter_type == 'table':
        # top
        position.append([0.2779583675156155, 0.26275243409152355, 0.7777829205816456])
        orientation.append([-0.35845471352424463, 0.9005717039108276, 0.09117319958905871, 0.2284038586206499])
        # right
        position.append([0.3794928027709814, -0.28253295485133534, 0.5972795770268642])
        orientation.append([-0.4325898269207546, 0.824556291103363, 0.1981843983729342, 0.30609785337529444])
        # right extended
        position.append([0.7097662955504827, -0.24557837079110972, 0.29336527812927965])
        orientation.append([-0.4355347391477553, 0.7903707027435303, 0.3665027905637759, 0.22649359856253057])
        # left
        position.append([0.24296657436741034, 0.6076877409200119, 0.6556095117823818])
        orientation.append([-0.13007583841209175, 0.9581239223480225, 0.036903300895661385, 0.2524221221852935])
        # left extended
        position.append([-0.12183199428806823, 0.7319461018461745, 0.43716734779267535])
        orientation.append([-0.0878360293412678, 0.9119853973388672, -0.03003669288141751, 0.39958138162624685])
        # top tilted
        position.append([0.24305885822630685, 0.33263431795949716, 0.6555593633787609])
        orientation.append([-0.655489850536862, 0.6997860074043274, -0.055496967900669915, 0.2784827853564631])

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'

    count = args.start_count
    while True:
        input('Press Enter to start collecting sample ' +  str(f"{count:06}"))
        rospy.loginfo('Collecting Sample ' + str(f"{count:06}"))
        sample_dir = args.dataset_path + '/' + f"{count:06}"
        if os.path.exists(sample_dir):
            rospy.logerr("Sample number already exist. Count: " + str(count))
            rospy.logerr("Need manual help, Exiting ...")
            exit()
        else:
            os.mkdir(sample_dir)
            os.mkdir(sample_dir + '/rgb')
            os.mkdir(sample_dir + '/depth')
            os.mkdir(sample_dir + '/cloud')

        tf_trans = {}

        for i in range(len(position)):
            # start cloud saver
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            cloud_saver_cli_args[3] = '_prefix:=' + sample_dir + '/cloud/' + f"{i:06}" + '_'
            cloud_saver_node = roslaunch.core.Node('pcl_ros', 'pointcloud_to_pcd', args=' '.join(cloud_saver_cli_args))
            process_cloud = launch.launch(cloud_saver_node)
            rgb_sub = rospy.Subscriber('/zivid_camera/color/image_color', Image, rgb_callback, (sample_dir, i))
            depth_sub = rospy.Subscriber('/zivid_camera/depth/image', Image, depth_callback, (sample_dir, i))

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

            rospy.sleep(2)  # wait camera capture frame and point cloud saver saves it

            process_cloud.stop()
            rgb_sub.unregister()
            depth_sub.unregister()

            tf_view = list()
            (trans, rot) = listener.lookupTransform('/iiwa_link_0', '/zivid_optical_frame', rospy.Time(0))
            tf_view.append({"source_frame": "iiwa_link_0", "target_frame": "zivid_optical_frame",
                             "translation": {"x": trans[0], "y": trans[1], "z": trans[2]},
                             "rotation_quaternion": {"x": rot[0], "y": rot[1], "z": rot[2], "w": rot[3]}})
            (trans, rot) = listener.lookupTransform('/iiwa_link_0', scene_link, rospy.Time(0))
            tf_view.append({"source_frame": "iiwa_link_0", "target_frame": "scene_link",
                             "translation": {"x": trans[0], "y": trans[1], "z": trans[2]},
                             "rotation_quaternion": {"x": rot[0], "y": rot[1], "z": rot[2], "w": rot[3]}})
            (trans, rot) = listener.lookupTransform('/zivid_optical_frame', scene_link, rospy.Time(0))
            tf_view.append({"source_frame": "zivid_optical_frame", "target_frame": "scene_link",
                             "translation": {"x": trans[0], "y": trans[1], "z": trans[2]},
                             "rotation_quaternion": {"x": rot[0], "y": rot[1], "z": rot[2], "w": rot[3]}})
            tf_trans[str(i)] = tf_view

            rospy.sleep(1)

        tf_json = json.dumps(tf_trans)
        json_file = open(sample_dir + '/scene_transformations.json', 'w')
        json_file.write(tf_json)
        json_file.close()


        count+=1


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
