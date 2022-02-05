import copy
import glob
import os.path

import cv2
import numpy as np

import shutil

dataset_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/test'

def show_image(img):
    #img_small = cv2.resize(img, (int(img.shape[1]/2), int(img.shape[0]/2)))
    img_small = img
    cv2.imshow("img", img_small)
    cv2.waitKey()
    cv2.destroyAllWindows()

for sample in glob.glob(dataset_path+'/*'):
    print(sample)
    os.makedirs(os.path.join(sample, 'depth_mm'), exist_ok=True)
    for depth_img_path in glob.glob(sample+'/depth/*'):
        print(depth_img_path)
        depth_img = cv2.imread(depth_img_path,-1)
        depth_img = depth_img / 10
        depth_img = depth_img.astype(np.uint16)

        cv2.imwrite(os.path.join(sample, 'depth_mm', depth_img_path[-10:]), depth_img)

        #show_image(depth_img)

        mask = np.zeros(depth_img.shape, dtype=np.uint8)
        mask[depth_img == 0] = 255

        #show_image(mask)

        inpainted_img = cv2.inpaint(depth_img, mask, inpaintRadius=100, flags=cv2.INPAINT_TELEA)

        #show_image(inpainted_img)

        cv2.imwrite(depth_img_path, inpainted_img)
