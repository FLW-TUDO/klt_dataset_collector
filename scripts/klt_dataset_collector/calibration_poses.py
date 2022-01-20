#!/usr/bin/python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2

from zivid_settings_from_file import *

def main():
    np.set_printoptions(linewidth=400)

    rospy.init_node('data_collector', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    rate = rospy.Rate(0.1)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'iiwa_link_0'
    
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    position = list()
    orientation = list()

    # 1
    position.append([0.2883114887885124, -0.0021990728958847586, 0.7270364365150974])
    orientation.append([-0.004469322082966892, 0.9960458278656006, -0.0009941283198063556, 0.08872305990397326])
    # 2
    position.append([0.7107196700842576, -0.010927907990982396, 0.5076358858169869])
    orientation.append([-0.004685097872451218, 0.9988821148872375, -0.009805712305349329, 0.0460050370692138])
    # 3
    position.append([0.22645405759330814, 0.00016444886036783097, 0.9761629080063953])
    orientation.append([0.00823908580866931, 0.9598917961120605, 0.007138206183637295, 0.28015873148547055])
    # 4
    position.append([0.6642890978686001, -0.28309946208180325, 0.44885448919060666])
    orientation.append([-0.3518809136618783, 0.9295275211334229, -0.023150171094924375, 0.10780755714457357])
    # 5
    position.append([0.20791279168148194, 0.5790008514260205, 0.5718865165425695])
    orientation.append([-0.426551675345418, 0.8104821443557739, -0.37174210567019345, 0.15159210547824836])
    # 6
    position.append([0.28164734875004266, 0.6294763338989938, 0.1340691015578778])
    orientation.append([0.9200633764266968, 0.17110117223225818, 0.22011961008189218, 0.27523650041454367])
    # 7
    position.append([0.7684930715064793, 0.12080297495597049, 0.17676987576456799])
    orientation.append([0.9829511046409607, 0.04441200672534445, 0.16324394135249665, 0.07201477556344449])
    # 8
    position.append([0.850533075073704, 0.15488533369146554, 0.2197323018850706])
    orientation.append([0.9833790063858032, 0.05462107276680355, 0.16055943444769175, 0.06483041737194462])
    # 9
    position.append([0.8233808629171018, -0.11007841710166498, 0.3359357303921017])
    orientation.append([0.9507935643196106, -0.2880232561940627, 0.08308841940534206, 0.07829786515230248])
    # 10
    position.append([0.6185055234985145, -0.4643385077623673, 0.2250001532518916])
    orientation.append([-0.45447058309696803, 0.8438899517059326, 0.23677301747020152, 0.15888606578577916])
    # 11
    position.append([0.5813542785366211,-0.48957048589957264, -0.09850311760802481])
    orientation.append([0.8038640022277832, 0.45041985298037057, 0.38638964804148856, -0.040343963315293184])
    # 12
    position.append([0.34573354260583783, 0.598606742846214, -0.10806808041357117])
    orientation.append([0.7316998243331909, 0.6006838278187645, -0.054349683543883, 0.3175537583157671])
    # 13
    position.append([0.653966303718863, 0.3916834478716141, -0.12949275619231873])
    orientation.append([0.8857424855232239, 0.42670773623355057, 0.08713615390481165, 0.16058647412382923])
    # 14
    position.append([0.6710875474609695, 0.45525331487654824, 0.45191995925342127])
    orientation.append([0.937463641166687, 0.33477896179350924, 0.09517647497883268, 0.005137562170093658])
    # 15
    position.append([0.5249970432472979, 0.5488184621953514, 0.3331299257242851])
    orientation.append([0.04721264898399929, 0.9144604802131653, -0.36402592554231566, 0.17034708036744772])
    # 16
    position.append([0.5526726779138548, -0.4387746396152108, -0.09703078411266751])
    orientation.append([-0.2824936352471634, 0.8743230104446411, 0.2913587023675044, 0.2662081077702107])

    calibration_dir = '/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/calibration_data/calibration_*_*'

    # Setting Zivid camera settings
    app = zivid.Application()
    camera = app.connect_camera()
    path = Path("/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/calibration/calibration_board_detection_settings.yml")
    settings = get_settings_from_yaml(path)

    for i in range(len(position)):
        input("Press enter for next position?")
        msg.header.seq = i
        msg.pose.position.x = position[i][0]
        msg.pose.position.y = position[i][1]
        msg.pose.position.z = position[i][2]
        msg.pose.orientation.x = orientation[i][0]
        msg.pose.orientation.y = orientation[i][1]
        msg.pose.orientation.z = orientation[i][2]
        msg.pose.orientation.w = orientation[i][3]
        pub.publish(msg)
        rospy.loginfo("Pose "+str(i+1))
        tros = tf.TransformerROS()
        matrix = np.array(tros.fromTranslationRotation(tuple(np.array(position[i])*1000), tuple(orientation[i])))

        matrix_dir = calibration_dir + '/pos' + f'{i+1:02}' + '.yaml'
        f = cv2.FileStorage(matrix_dir, flags=1)
        f.write(name='PoseState', val=matrix)
        f.release()

        input("did robot reach position? [Press enter]")

        frame = camera.capture(settings)
        frame.save(calibration_dir + '/img' + f'{i+1:02}' + '.zdf')

        #rospy.sleep(10)

    # bash command to calculate calibration
    # ZividExperimentalHandEyeCalibration --eih -d $dataset --tf $dataset/tf.yml --rf $dataset/rf.yml

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
