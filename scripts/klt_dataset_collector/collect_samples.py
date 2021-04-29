#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

from zivid_camera.srv import *

import ros_numpy

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
        max_capture_time = rospy.Duration.from_sec(0.20)
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
    rospy.init_node('data_collector', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    #while pub.get_num_connections() == 0:
    #    rospy.sleep(0.1)

    # setup zivid
    s = Sample()

    position = list()
    orientation = list()
    # capture positions
    # 1
    position.append([0.6185055234985145, -0.4643385077623673, 0.2250001532518916])
    orientation.append([-0.45447058309696803, 0.8438899517059326, 0.23677301747020152, 0.15888606578577916])
    # 2
    position.append([0.653966303718863, 0.3916834478716141, -0.12949275619231873])
    orientation.append([0.8857424855232239, 0.42670773623355057, 0.08713615390481165, 0.16058647412382923])

    #msg = PoseStamped()
    #msg.header.stamp = rospy.Time.now()
    #msg.header.frame_id = 'iiwa_link_0'
    for i in range(len(position)):
        #msg.header.seq = i
        #msg.pose.position.x = position[i][0]
        #msg.pose.position.y = position[i][1]
        #msg.pose.position.z = position[i][2]
        #msg.pose.orientation.x = orientation[i][0]
        #msg.pose.orientation.y = orientation[i][1]
        #msg.pose.orientation.z = orientation[i][2]
        #msg.pose.orientation.w = orientation[i][3]
        #pub.publish(msg)
        #rospy.sleep(10) # wait till robot reach goal position
        s.capture_assistant_suggest_settings()
        s.capture()
        pc2_msg = rospy.wait_for_message("/zivid_camera/points/xyzrgba", PointCloud2)

        #gen = pc2.read_points(pc2_msg, skip_nans=True, field_names=("x", "y", "z"))
        #print(gen)

        for p in pc2.read_points(pc2_msg, field_names = ("x", "y", "z", "rgba"), skip_nans=False):
            print(" x : %f  y: %f  z: %f rgba: %f" %(p[0],p[1],p[2],p[3]))
            len(p[3])

        #rospy.sleep(2) # wait camera capture frame


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass