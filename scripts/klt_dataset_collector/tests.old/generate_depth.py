import glob

import cv2
import numpy as np

path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/saeweqwerqwer'
directories = glob.glob(path+'/000*')

for folder in directories:
    depth_path = glob.glob(folder+ '/depth/*.png')
    for img_path in depth_path:
        img = cv2.imread(img_path,-1)

        img = img / 10
        img = img.astype('uint8')

        #img = img * 10
        #img = img.astype('uint16')

        cv2.imwrite(img_path, img)
