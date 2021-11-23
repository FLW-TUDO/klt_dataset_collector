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
    depth_img = bridge.imgmsg_to_cv2(msg, "16UC1") * 10000  # convert resolution from meter to 1/10 mm
    # Note depth scale in BOP is 0.1
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
        # 1 - top
        position.append([0.40010042658960016, -0.40230712128976354, 0.586433233868703])
        orientation.append([0.38184669676554683, 0.9182227253913879, -0.040387323075866455, 0.0971030723192087])
        # right
        position.append([0.09995268780379314, -0.6798327066524414, 0.42276478325101163])
        orientation.append([0.41812238815860736, 0.8724203109741211, 0.1642862263735469, 0.19252671297070037])
        # right extended
        position.append([-0.08852583692331335, -0.7838423649695757,  0.38278872618017634])
        orientation.append([-0.1256997951229335, 0.9399797916412354, -0.03335736578791394, 0.3154756243896394])
        # left
        position.append([0.5965843092650834, 0.10297097492013302, 0.480372713413665])
        orientation.append([0.28916164994160787, 0.8849726915359497, -0.3505103786634116, 0.10174163157915642])
        # left extended
        position.append([0.7724643922975897, 0.12095998532630636,  0.32069815446447525])
        orientation.append([0.5691317843938981, 0.7515872716903687, -0.26892910381104734, 0.19718711725501137])
        # top tilted
        position.append([0.41003542694620493, -0.33176993388317294, 0.6930441593796396])
        orientation.append([0.8800539970397949, 0.4570231629515724, 0.0528199449492035, 0.11766420343976217])
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
        os.system('clear')
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
