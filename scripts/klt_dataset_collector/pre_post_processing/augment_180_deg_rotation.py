import glob
import os

import cv2
import numpy as np
import json
import open3d as o3d

def main():
    # loop samples
    samples_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/test'


    # augment annotations
    for sample_path in glob.glob(samples_path + '/*'):
        print(sample_path)
        gt_path = os.path.join(sample_path, 'scene_gt.json')
        with open(gt_path, "r") as gt_file:
            gt_scene = json.load(gt_file)
            for view in gt_scene:
                objects = gt_scene[view]
                for obj_num in range(len(objects)):
                    object = objects[obj_num]
                    R1 = np.array(object['cam_R_m2c'])
                    t1 = np.array(object['cam_t_m2c'])
                    transform_cam1_to_scene = np.vstack((np.hstack((R1, t1[:, None])), [0, 0, 0, 1]))
                    R2 = o3d.geometry.get_rotation_matrix_from_xyz(np.array((0,0,np.pi)))
                    t2 = np.array([5, 35, 0])
                    transform_cam2_to_cam1 = np.vstack((np.hstack((R2, t2[:, None])), [0, 0, 0, 1]))
                    transform_cam2_to_scene = np.matmul(transform_cam2_to_cam1, transform_cam1_to_scene)
                    R3 = transform_cam2_to_scene[0:3,0:3]
                    t3 = transform_cam2_to_scene[0:3, 3]
                    gt_scene[view][obj_num]['cam_R_m2c'] = R3.tolist()
                    gt_scene[view][obj_num]['cam_t_m2c'] = t3.tolist()
        with open(gt_path, "w") as gt_file:
            json.dump(gt_scene, gt_file)

        # augment images
        rgb_path = os.path.join(sample_path, 'rgb')
        for img_path in glob.glob(rgb_path+'/*'):
            img = cv2.imread(img_path)
            img = cv2.rotate(img, cv2.ROTATE_180)
            cv2.imwrite(img_path, img)
        depth_path = os.path.join(sample_path, 'depth')
        for img_path in glob.glob(depth_path+'/*'):
            img = cv2.imread(img_path,-1)
            img = cv2.rotate(img, cv2.ROTATE_180)
            cv2.imwrite(img_path, img)
        mask_path = os.path.join(sample_path, 'mask')
        for img_path in glob.glob(mask_path+'/*'):
            img = cv2.imread(img_path,-1)
            img = cv2.rotate(img, cv2.ROTATE_180)
            cv2.imwrite(img_path, img)
        mask_visib_path = os.path.join(sample_path, 'mask_visib')
        for img_path in glob.glob(mask_visib_path+'/*'):
            img = cv2.imread(img_path,-1)
            img = cv2.rotate(img, cv2.ROTATE_180)
            cv2.imwrite(img_path, img)


if __name__ == '__main__':
    main()
