#!/usr/bin/env python
import copy

from gripper.onrobot_vgc10 import OnRobotVGC10

import rospy
import roslaunch
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tr
from cv_bridge import CvBridge, CvBridgeError
import rospkg

import cv2
import numpy as np
import open3d as o3d

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

from zivid_camera.srv import *

from agnostic_segmentation_live_demo import agnostic_segmentation

from vector_quaternion import pose_from_vector3D

bridge = CvBridge()

rgb_img = None
depth_img = None

from geometry_msgs.msg import Pose


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

    drop_position = PoseStamped()
    drop_position.header.stamp = rospy.Time.now()
    drop_position.pose.position.x = 0.3801143215460226
    drop_position.pose.position.y = -0.3841013711418166
    drop_position.pose.position.z = 0.759381750959396
    drop_position.pose.orientation.x = 0.35727852968808477
    drop_position.pose.orientation.y = 0.9106038808822632
    drop_position.pose.orientation.z = -0.07019330871685607
    drop_position.pose.orientation.w = 0.19551352664641677

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
        #model_path = '/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/agnostic_segmentation_live_demo/agnostic_segmentation_model.pth'
        #seg_img, predictions = agnostic_segmentation.segment_image(rgb_img, rgb_img, model_path)
        model_path = '/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/pipeline/FAT_5k_ML2R_0.5k.pth'
        predictions = segmentor.segment_image(rgb_img, model_path)
        seg_img = segmentor.draw_segmented_image(rgb_img, predictions)

        #cv2.imshow('Segmented image', seg_img)
        #cv2.imshow('Segmented image', cv2.resize(seg_img, (0,0), fx=0.5, fy=0.5))
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        c_matrix = [1778.81005859375, 0.0, 967.9315795898438, 0.0, 1778.870361328125, 572.4088134765625, 0.0, 0.0, 1.0]
        c_matrix = [float(i) for i in c_matrix]
        c_matrix = np.array(c_matrix).reshape((3, 3))

        suction_pts = grasper.compute_suction_points(rgb_img, depth_img, c_matrix, predictions)
        #suction_pts_image = grasper.visualize_suction_points(rgb_img, c_matrix, suction_pts)

        #cv2.imshow('suction points', suction_pts_image)
        #cv2.imshow('suction points', cv2.resize(suction_pts_image, (0,0), fx=0.5, fy=0.5))
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        # use first suction point
        suction_pt = suction_pts[0]

        grasp = PoseStamped()
        grasp.header.stamp = rospy.Time.now()
        grasp.header.frame_id = 'zivid_optical_frame'
        grasp.header.stamp = rospy.Time.now()
        grasp.pose.position.x = suction_pt[0][0]
        grasp.pose.position.y = suction_pt[0][1]
        grasp.pose.position.z = suction_pt[0][2]
        grasp.pose.orientation.x = suction_pt[1][0]
        grasp.pose.orientation.y = suction_pt[1][1]
        grasp.pose.orientation.z = suction_pt[1][2]
        grasp.pose.orientation.w = suction_pt[1][3]

        #tmp_trans = copy.copy(grasp_o.pose.position) # shift to origin - rotate - then back to original position
        #grasp_o.pose.position.x = 0
        #grasp_o.pose.position.y = 0
        #grasp_o.pose.position.z = 0
        ## grasp looking to x axis - change it to z (-90 around Z)
        #transform = TransformStamped()
        #transform.child_frame_id = 'zivid_optical_frame'
        #transform.header.frame_id = 'zivid_optical_frame'
        #transform.header.stamp = rospy.Time.now()
        #transform.transform.translation.x = 0
        #transform.transform.translation.y = 0
        #transform.transform.translation.z = 0
        #q = tr.quaternion_from_euler(0, -np.pi / 2, 0, 'rxyz')
        #transform.transform.rotation.x = q[0]
        #transform.transform.rotation.y = q[1]
        #transform.transform.rotation.z = q[2]
        #transform.transform.rotation.w = q[3]
        #grasp_o = tf2_geometry_msgs.do_transform_pose(grasp, transform)
        #grasp_o.pose.position = tmp_trans
        # -------------------------------------------------------------------
        # -------------------------------------------------------------------
        def msg_to_se3(pose_stamped_msg):
            msg = pose_stamped_msg.pose
            p = np.array([msg.position.x, msg.position.y, msg.position.z])
            q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            norm = np.linalg.norm(q)
            if np.abs(norm - 1.0) > 1e-3:
                raise ValueError("Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(str(q), np.linalg.norm(q)))
            elif np.abs(norm - 1.0) > 1e-6:
                q = q / norm
            g = tr.quaternion_matrix(q)
            g[0:3, -1] = p
            return g

        iiwa_base_transform = tf_buffer.lookup_transform("iiwa_link_0", grasp.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        grasp_o = tf2_geometry_msgs.do_transform_pose(grasp, iiwa_base_transform)


        H_base_tcp = msg_to_se3(grasp_o)
        H_tcp_ee = tf_buffer.lookup_transform("iiwa_link_ee", "vgc10_tcp", rospy.Time(0), rospy.Duration(1.0))
        # TODO change distance to read from TF transform
        H_tcp_ee = np.array([[1,0,0,-H_tcp_ee.transform.translation.z],[0,1,0,H_tcp_ee.transform.translation.y],[0,0,1,H_tcp_ee.transform.translation.x],[0,0,0,1]])
        #H_tcp_ee = np.array([[1,0,0,H_tcp_ee.transform.translation.x],[0,1,0,H_tcp_ee.transform.translation.y],[0,0,1,],[0,0,0,1]])

        H_base_ee = np.matmul(H_base_tcp, H_tcp_ee)
        grasp_ee = PoseStamped()
        grasp_ee.header.stamp = rospy.Time.now()
        grasp_ee.header.frame_id = 'iiwa_link_0'
        grasp_ee.header.stamp = rospy.Time.now()
        print(H_base_ee[0:3,3])
        grasp_ee.pose.position.x = H_base_ee[0,3]
        grasp_ee.pose.position.y = H_base_ee[1,3]
        grasp_ee.pose.position.z = H_base_ee[2,3]
        q = tr.quaternion_from_matrix(H_base_ee)
        print(q)
        grasp_ee.pose.orientation.x = q[0]
        grasp_ee.pose.orientation.y = q[1]
        grasp_ee.pose.orientation.z = q[2]
        grasp_ee.pose.orientation.w = q[3]

        # -------------------------------------------------------------------
        # -------------------------------------------------------------------

        # move to pre grasp position
        pub.publish(pre_grasp_point)
        rospy.sleep(10)  # wait till robot reach goal position

        # start suction and move grasp point
        gripper.set_pressure_channel_A(80)
        pub.publish(grasp_point)
        rospy.sleep(5)

        # back to pre grasp position
        pub.publish(pre_grasp_point)
        rospy.sleep(5)  # wait till robot reach goal position

        # move to 2nd bin position
        pub.publish(drop_position)
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
