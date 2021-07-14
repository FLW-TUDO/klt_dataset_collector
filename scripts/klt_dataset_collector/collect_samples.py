#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from zivid_camera.srv import *

import roslaunch
import argparse
import os


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


def main():
    # read arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", default='/home/iiwa/segmentation/flw_dataset')
    parser.add_argument("--start-count", type=int, default=0)
    args = parser.parse_args()

    if not os.path.exists(args.dataset_path):
        rospy.logerr("Error: dataset folder doesn't exist")
        exit()

    rospy.init_node('data_collector', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)

    # setup PCD cloud saver
    saver_package = 'pcl_ros'
    saver_node = 'pointcloud_to_pcd'
    point_cloud_topic = '/passthrough/box_filtered'
    frame = 'bin_link'
    cli_args = ['input:=' + point_cloud_topic,
                '_binary:=True',
                '_fixed_frame:=' + frame,
                '_prefix:=' + args.dataset_path]

    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # setup zivid
    s = Sample()

    position = list()
    orientation = list()
    # capture positions
    # 1 - top
    position.append([0.4536302081007271, 0.00014568127071799047, 0.465632596695722])
    orientation.append([0.00033934633884419606, 0.9884247183799744, -2.754895333484324e-05, 0.1517116969851703])
    # 2 - right
    position.append([0.598213268524855, -0.46227607401174414, 0.3893305460186694])
    orientation.append([-0.09238601030833919, 0.9627739191055298, 0.2428643130344425, 0.07448597649883315])
    # 3 - left
    position.append([0.5921051907749318, 0.467128419984067, 0.40259435352400785])
    orientation.append([-0.10990742800801294, 0.9505132436752319, -0.2813433794214693, 0.0727384796493719])

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'

    count = args.start_count
    sample_dir = args.dataset_path + '/' + f"{count:05}"
    if os.path.exists(sample_dir):
        rospy.logerr("Sample number already exist. Count: " + str(count))
        rospy.logerr("Need manual help, Exiting ...")
        exit()
    else:
        os.mkdir(sample_dir)

    for i in range(len(position)):
        # start cloud saver
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        cli_args[3] = '_prefix:=' + sample_dir + '/cloud_' + str(i) + '_'
        node = roslaunch.core.Node(saver_package, saver_node, args=' '.join(cli_args))
        process = launch.launch(node)

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
        process.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
