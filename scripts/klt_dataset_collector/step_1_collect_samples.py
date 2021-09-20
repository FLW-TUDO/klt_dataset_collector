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
    cv2.imwrite(args[0]+ '/color_' + str(args[1]) + '.png', rgb_img)

def depth_callback(msg, args):
    depth_img = bridge.imgmsg_to_cv2(msg, "32FC1") * 100  # multiply by 100 to convert to cm
    cv2.imwrite(args[0]+ '/depth_' + str(args[1]) + '.png', depth_img)

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
        # 1 - top
        position.append([0.4536302081007271, 0.00014568127071799047, 0.465632596695722])
        orientation.append([0.00033934633884419606, 0.9884247183799744, -2.754895333484324e-05, 0.1517116969851703])
        # 2 - right
        position.append([0.598213268524855, -0.46227607401174414, 0.3893305460186694])
        orientation.append([-0.09238601030833919, 0.9627739191055298, 0.2428643130344425, 0.07448597649883315])
        # 3 - left
        position.append([0.5921051907749318, 0.467128419984067, 0.40259435352400785])
        orientation.append([-0.10990742800801294, 0.9505132436752319, -0.2813433794214693, 0.0727384796493719])
    elif args.filter_type == 'table':
        # 1 - top tilted
        position.append([0.45366535956448795, 0.00019003750267646333, 0.7788801811010954])
        orientation.append([0.0014396677025957014, 0.9728067517280579, -0.00028660261442955726, 0.23161359892331101])
        # 2 - right
        position.append([0.44258463088703054, -0.4911573773280114, 0.6543658294324605])
        orientation.append([0.2552882853983005, 0.9138663411140442, 0.2506485566196609, 0.1919673573426694])
        # 3 - left
        position.append([0.3179112403677761, 0.5951135610348454, 0.34818191358262757])
        orientation.append([0.17968118618225037, 0.9074693322181702, -0.07799770812562887, 0.3716590689952614])
        # right down
        position.append([0.6455294403996836, -0.42170832216318355, -0.008022791903315238])
        orientation.append([0.3345335858576662, 0.7673130035400391, 0.515384421634103, 0.18356725324507037])
        # left down
        position.append([0.620483350232771, 0.4967904848477807, 0.019487615238156706])
        orientation.append([-0.5770074081867963, 0.617912232875824, -0.5260044420860153, 0.09255396959828174])

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'

    count = args.start_count
    while True:
        os.system('clear')
        tcflush(sys.stdin, TCIFLUSH)
        input('Press Enter to start collecting sample ' +  str(f"{count:05}"))
        rospy.loginfo('Collecting Sample ' + str(f"{count:05}"))
        sample_dir = args.dataset_path + '/' + f"{count:05}"
        if os.path.exists(sample_dir):
            rospy.logerr("Sample number already exist. Count: " + str(count))
            rospy.logerr("Need manual help, Exiting ...")
            exit()
        else:
            os.mkdir(sample_dir)

        tf_trans = {}

        for i in range(len(position)):
            # start cloud saver
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            cloud_saver_cli_args[3] = '_prefix:=' + sample_dir + '/cloud_' + str(i) + '_'
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
