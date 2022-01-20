import glob
import os
import cv2
import numpy as np
import json
from skimage.util import random_noise


def main():
    # loop samples
    samples_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/test'


    # augment annotations
    for sample_path in glob.glob(samples_path + '/*'):
        print(sample_path)

        gt_path = os.path.join(sample_path, 'scene_gt.json')
        with open(gt_path, "r") as gt_file:
            gt_scene = json.load(gt_file)

            # loop samples in scene
            for view in gt_scene:
                print("Scene: " + view)
                rgb_img_path = sample_path + '/rgb/' +  f'{int(view):06}' + '.png'
                rgb_img = cv2.imread(rgb_img_path, -1)
                # Add salt-and-pepper noise to the image.
                rgb_img = random_noise(rgb_img, mode='s&p', amount=0.1)
                rgb_img = np.array(255 * rgb_img, dtype='uint8')
                cv2.imwrite(rgb_img_path, rgb_img)

                # mask out Depth
                depth_img_path = sample_path + '/depth/' +  f'{int(view):06}' + '.png'
                depth_img = cv2.imread(depth_img_path, -1)
                indices = np.random.choice(np.arange(depth_img.size), replace=False, size=int(depth_img.size * 0.2))
                tmp_img = depth_img.flatten()
                tmp_img[indices] = 0
                depth_img = tmp_img.reshape(depth_img.shape)
                cv2.imwrite(depth_img_path, depth_img)


if __name__ == '__main__':
    main()
