import numpy as np
import os
import json
import cv2
import random
import matplotlib.pyplot as plt
import argparse
import time
import pycocotools
from math import ceil
import copy
from detectron2.utils.logger import setup_logger
from detectron2 import model_zoo
from detectron2.engine import DefaultTrainer, DefaultPredictor, launch
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.structures import BoxMode
from detectron2.utils.visualizer import ColorMode

setup_logger()  # initialize the detectron2 logger and set its verbosity level to “DEBUG”.


def save_infered_images(save_path, folder_name, img, img_idx, enable):
    if enable:
        try:
            save_path = os.path.join(save_path, folder_name)
            os.makedirs(save_path, exist_ok=True)
            cv2.imwrite(f"{save_path}/{img_idx}.jpg", img)
        except:
            print("something went wrong")
    else:
        pass


def main():

    img_name = r"/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/demo/img/color_0.png"
    save_dir = r"/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/demo/img"

    confidence = 0.95

    # --- var dec ---

    MetadataCatalog.get("kitchen0_val").set(thing_classes=["object"])
    kitchen0_metadata = MetadataCatalog.get("kitchen0_val")

    # --- Config setup ---

    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))  # WAS 3x.y
    cfg.MODEL.WEIGHTS = '/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/demo/model_final.pth'
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
    cfg.MODEL.SEM_SEG_HEAD.NUM_CLASSES = 1
    cfg.MODEL.ROI_BOX_HEAD.CLS_AGNOSTIC_BBOX_REG = True
    cfg.MODEL.ROI_MASK_HEAD.CLS_AGNOSTIC_MASK = True
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = confidence
    predictor = DefaultPredictor(cfg)

    start = time.time()
    im = cv2.imread(img_name, cv2.IMREAD_COLOR)
    outputs = predictor(im)
    v = Visualizer(im,
                   metadata=kitchen0_metadata,
                   scale=0.5,
                   instance_mode=ColorMode.IMAGE
                   )
    out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    img = out.get_image()

    # save_infered_images(save_path=save_dir, folder_name="results", img=img, img_idx=img_idx, enable=True)

    end = time.time()
    print("Inference time: ", end - start)

    cv2.imshow("Infer", img)
    cv2.waitKey(0)



if __name__ == '__main__':
    main()
