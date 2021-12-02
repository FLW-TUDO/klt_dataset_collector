import copy
import glob
import os
import random
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

            # loop samples in scene
            for view in gt_scene:
                print("Scene: " + view)
                objects = gt_scene[view]
                # choose 50% of them to remove
                objects_to_remove = random.sample(range(0,len(objects)), int(len(objects)/2))
                print(objects_to_remove)
                for remove_obj_num in objects_to_remove:
                    print(remove_obj_num)
                    #gt_scene[view].pop(remove_obj_num)
                    # load visib mask
                    visib_img_path = sample_path + '/mask_visib/' +  f'{int(view):06}' + '_' + f'{remove_obj_num:06}' + '.png'
                    visib_img = cv2.imread(visib_img_path, -1)
                    rgb_img_path = sample_path + '/rgb/' +  f'{int(view):06}' + '.png'
                    rgb_img = cv2.imread(rgb_img_path, -1)
                    rgb_visib_mask = copy.deepcopy(visib_img)
                    #rgb_visib_mask = cv2.bitwise_not(rgb_visib_mask)
                    #rgb_visib_mask[rgb_visib_mask==255] = 170
                    #rgb_visib_mask = visib_img * (220/255)
                    #rgb_visib_mask = rgb_visib_mask.astype('uint8')
                    #rgb_visib_mask = cv2.bitwise_not(rgb_visib_mask)

                    #cv2.imshow('image window', rgb_visib_mask)
                    #cv2.waitKey(0)
                    #cv2.destroyAllWindows()

                    #rgb_img = cv2.bitwise_or(rgb_img, rgb_img, mask=rgb_visib_mask)
                    rgb_img[visib_img == 255] = 120
                    cv2.imwrite(rgb_img_path, rgb_img)
                    # mask out Depth
                    depth_img_path = sample_path + '/depth/' +  f'{int(view):06}' + '.png'
                    depth_img = cv2.imread(depth_img_path, -1)
                    depth_img_mask = cv2.bitwise_not(visib_img)
                    depth_img = cv2.bitwise_or(depth_img, depth_img, mask=depth_img_mask)
                    cv2.imwrite(depth_img_path, depth_img)
                    # delete mask and visib_mask
                    # os.remove(visib_img_path) # TODO uncomment
                    mask_img_path = sample_path + '/mask/' + f'{int(view):06}' + '_' + f'{remove_obj_num:06}' + '.png'
                    # os.remove(mask_img_path) # TODO uncomment
                for i in sorted(objects_to_remove, reverse=True):
                    del gt_scene[view][i]
            with open(gt_path, "w") as gt_file:
                json.dump(gt_scene, gt_file)



if __name__ == '__main__':
    main()
