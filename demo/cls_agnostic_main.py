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

'''
def get_obj_dicts(img_dir, json_file_name):
    json_file = os.path.join(img_dir, json_file_name)  # get the corresponding json file

    with open(json_file) as f:  # , encoding='utf-8-sig')
        imgs_anns = json.load(f)  # load image annotations

    # the keys list is important as the keys are the different subdirectories needed to access the imgs
    keys_list = []
    for key, val in imgs_anns.items():
        keys_list.append(key.split(".")[-1])

    dataset_dicts = []
    for idx, v in enumerate(imgs_anns.values()):
        sub_dir_name = keys_list[idx]
        current_img_dir = os.path.join(img_dir, sub_dir_name)
        # print("currently at: ", current_img_dir, v["filename"])
        record = {}

        filename = os.path.join(current_img_dir, v["filename"].split(".jpg")[0] + ".depth.png" )
        height, width = cv2.imread(filename).shape[:2]

        record["file_name"] = filename
        record["image_id"] = idx
        record["height"] = height
        record["width"] = width

        with open(filename.split('.depth.png')[0] + '.json') as f:
            img_json_file = json.load(f)

        annos = v["regions"]
        objs = []
        for id, item_anno in annos.items():  # _ just indicates that we don't care about that var
            assert not item_anno["region_attributes"]
            current_class = id
            for l in range(len(item_anno["shape_attributes"]["all_points_x"])):
                anno = item_anno["shape_attributes"]

                px = anno["all_points_x"][l]
                py = anno["all_points_y"][l]

                px = [np.round(x) for x in px]
                py = [np.round(x) for x in py]
                if not px or not py:
                    continue
                poly = [(x + 0.5, y + 0.5) for x, y in zip(px, py)]  # [(x + 0.5, y + 0.5) for x, y in zip(px, py)]
                poly = [p for x in poly for p in x]

                object = img_json_file["objects"]
                BBox = []
                for ob_idx in range(len(object)):
                    if object[ob_idx]["class"] == current_class:
                        BBox.append(object[ob_idx]["bounding_box"])

                with open(os.path.join(img_dir, '_object_settings.json')) as f:
                    object_settings = json.load(f)
                    for n in range(len(object_settings["exported_objects"])):
                        if object_settings["exported_object_classes"][n] == current_class:
                            id = n
                obj = {
                    # "bbox": [np.min(px), np.min(py), np.max(px), np.max(py)],
                    "bbox": [BBox[l]["top_left"][1], BBox[l]["top_left"][0], BBox[l]["bottom_right"][1],
                             BBox[l]["bottom_right"][0]],
                    "bbox_mode": BoxMode.XYXY_ABS,
                    "segmentation": [poly],
                    "category_id": 0,  # should be =id if multi class not just obj
                }
                objs.append(obj)

        record["annotations"] = objs
        #        record["sem_seg_file_name"] = sem_seg_file_name_and_dir
        dataset_dicts.append(record)
    return dataset_dicts
'''


def get_obj_dicts(img_dir, json_file_name):
    json_file = os.path.join(img_dir, json_file_name)  # get the corresponding json file
    print("JSON being read: ", json_file)
    with open(json_file) as f:  # , encoding='utf-8-sig')
        imgs_anns = json.load(f)  # load image annotations

    # the keys list is important as the keys are the different subdirectories needed to access the imgs
    keys_list = []
    directory_name = []
    for key, val in imgs_anns.items():
        keys_list.append(key.split(".")[-1]) # The key is 000000.left.002_master_chef_can_16k.kitedemo_0 from which we took kitedemo_0
        directory_name.append(key.split(".")[-2]) # here it is the directory name like the class and so

    dataset_dicts = []
    for idx, v in enumerate(imgs_anns.values()):
        if idx == 70000:
            print("wee")
        sub_dir_name = os.path.join(directory_name[idx], keys_list[idx]) # this is masterchef/kitedemo_0 should be only kitedemo_0
        #sub_dir_name = keys_list[idx]  # for the evaluation script only!!
        current_img_dir = os.path.join(img_dir, sub_dir_name)
        #print("currently at: ", current_img_dir, v["filename"])
        record = {}

        #filename = os.path.join(current_img_dir, v["filename"])  # For running the RGBD version
        filename = os.path.join(current_img_dir, v["filename"].split(".png")[0] + ".jpg")  # To use RGBD JSON for RGB
        #filename = os.path.join(current_img_dir, v["filename"])  # evaluation only modification
        #sem_seg_file_name_and_dir = filename.split('.png')[0] + '.seg.png'
        sem_seg_file_name_and_dir = filename.split('.jpg')[0] + '.seg.png'
        #sem_seg_file_name_and_dir = filename.split('.png')[0] + '.seg.png'  # FOR evaluation
        mask = cv2.imread(sem_seg_file_name_and_dir, cv2.IMREAD_UNCHANGED)
        print(filename)
        height, width = cv2.imread(filename, cv2.IMREAD_UNCHANGED).shape[:2]

        record["file_name"] = filename
        record["image_id"] = idx
        record["height"] = height
        record["width"] = width

        #with open(filename.split('.png')[0] + '.json') as f:  # For the RGBD version
        with open(filename.split('.jpg')[0] + '.json') as f:  # For the RGB version
            img_json_file = json.load(f)
        annos = []
        for hhh in img_json_file["objects"]:
            annos.append(hhh["class"])
        # Order the class names in descending order and with no duplicates
        # -------------------------
        ordered_annos = []
        for ref in annos:
            ordered_annos.append(ref.split("_")[0])
        ordered_annos.sort(reverse=True)
        filtered_ordered_annos = []
        for _, o_a in enumerate(ordered_annos):
            for an in annos:
                if o_a == an.split("_")[0] and filtered_ordered_annos.count(an) == 0:
                    filtered_ordered_annos.append(an)
        # -------------------------
        # annos = v["regions"]
        objs = []
        for id_idx, id in enumerate(filtered_ordered_annos):  # _ just indicates that we don't care about that var
            # assert not item_anno["region_attributes"]
            current_class = id
            # ------------------------ FOR MIXED ITEMS ONLY ----------------------
            if directory_name[idx] == "mixed" or "png":
                # Get seg_bit value:
                with open(
                        r"/media/ghanem/3C448DDD448D99F2/facebook-detectron/data/splitted_FAT_full_rgb/train/_object_settings.json") as f:
                    object_settings = json.load(f)
                value = \
                    object_settings["exported_objects"][
                        object_settings["exported_object_classes"].index(current_class)][
                        "segmentation_class_id"]
                if mask.max() > value:
                    mask[mask == mask.max()] = 0  # in case of errors in the json we remove any not mentioned values
            # -------------------------------------------------------------------
            # Get how many times does a class exist in the img and its BBox
            object = img_json_file["objects"]
            BBox = []
            for ob_idx in range(len(object)):
                if object[ob_idx]["class"] == current_class:
                    BBox.append(object[ob_idx]["bounding_box"])

            for l in range(len(BBox)):

                # Get obj ROI and increase BBox margins to avoid clipping
                BBox_for_ROI = copy.deepcopy(BBox[l])
                # BBox_for_ROI["top_left"][0] = BBox_for_ROI["top_left"][0] + 0.9
                # BBox_for_ROI["top_left"][1] = BBox_for_ROI["top_left"][1] * 1.1
                # BBox_for_ROI["bottom_right"][0] = BBox_for_ROI["bottom_right"][0] * 1.1
                # BBox_for_ROI["bottom_right"][1] = BBox_for_ROI["bottom_right"][1] * 0.9
                # Some BBox value conditions in case they are out of range
                if BBox_for_ROI["top_left"][0] <= 0:
                    BBox_for_ROI["top_left"][0] = 0
                if BBox_for_ROI["top_left"][0] > height:
                    BBox_for_ROI["top_left"][0] = height
                if BBox_for_ROI["top_left"][1] <= 0:
                    BBox_for_ROI["top_left"][1] = 0
                if BBox_for_ROI["top_left"][1] > width:
                    BBox_for_ROI["top_left"][1] = width
                if BBox_for_ROI["bottom_right"][0] <= 0:
                    BBox_for_ROI["bottom_right"][0] = 0
                if BBox_for_ROI["bottom_right"][0] > height:
                    BBox_for_ROI["bottom_right"][0] = height
                if BBox_for_ROI["bottom_right"][1] <= 0:
                    BBox_for_ROI["bottom_right"][1] = 0
                if BBox_for_ROI["bottom_right"][1] > width:
                    BBox_for_ROI["bottom_right"][1] = width

                if ceil(BBox_for_ROI["top_left"][1]) == ceil(BBox_for_ROI["bottom_right"][1]):
                    if 0 < ceil(BBox_for_ROI["top_left"][1]) <= (width - 1):
                        BBox_for_ROI["top_left"][1] = BBox_for_ROI["top_left"][1] + 1
                    elif BBox_for_ROI["top_left"][1] == 0:
                        BBox_for_ROI["top_left"][1] = BBox_for_ROI["top_left"][1] + 1
                    elif ceil(BBox_for_ROI["top_left"][1]) == width:
                        BBox_for_ROI["bottom_right"][1] = BBox_for_ROI["bottom_right"][1] - 1

                if ceil(BBox_for_ROI["top_left"][0]) == ceil(BBox_for_ROI["bottom_right"][0]):
                    if 0 < ceil(BBox_for_ROI["top_left"][0]) <= (height - 1):
                        BBox_for_ROI["bottom_right"][0] = BBox_for_ROI["bottom_right"][0] + 1
                    elif BBox_for_ROI["top_left"][0] == 0:
                        BBox_for_ROI["bottom_right"][0] = BBox_for_ROI["bottom_right"][0] + 1
                    elif ceil(BBox_for_ROI["top_left"][0]) == height:
                        BBox_for_ROI["top_left"][0] = BBox_for_ROI["top_left"][0] - 1

                if BBox_for_ROI["top_left"][1] < BBox_for_ROI["bottom_right"][1]:
                    if BBox_for_ROI["top_left"][0] < BBox_for_ROI["bottom_right"][0]:
                        roi = mask[ceil(BBox_for_ROI["top_left"][0]):ceil(BBox_for_ROI["bottom_right"][0]),
                              ceil(BBox_for_ROI["top_left"][1]):ceil(BBox_for_ROI["bottom_right"][1])]
                    else:
                        roi = mask[ceil(BBox_for_ROI["bottom_right"][0]):ceil(BBox_for_ROI["top_left"][0]),
                              ceil(BBox_for_ROI["top_left"][1]):ceil(BBox_for_ROI["bottom_right"][1])]
                    # cv2.imshow("roi", roi)
                    # cv2.imshow("mask", mask)
                    # cv2.waitKey(0)
                else:
                    if BBox_for_ROI["top_left"][0] < BBox_for_ROI["bottom_right"][0]:
                        roi = mask[ceil(BBox_for_ROI["top_left"][0]):ceil(BBox_for_ROI["bottom_right"][0]),
                              ceil(BBox_for_ROI["bottom_right"][1]):ceil(BBox_for_ROI["top_left"][1])]
                    else:
                        roi = mask[ceil(BBox_for_ROI["bottom_right"][0]):ceil(BBox_for_ROI["top_left"][0]),
                              ceil(BBox_for_ROI["bottom_right"][1]):ceil(BBox_for_ROI["top_left"][1])]

                # Set any other obj inside the ROI to zero
                if directory_name[idx] == "mixed" or "png":
                    _, threshed_roi = cv2.threshold(roi, value - 1, 255, 0)

                else:
                    threshed_roi = roi
                # Create a zeros matrix with the img HW and add the item to it with its original pos
                one_obj_mask = np.zeros((height, width), 'uint8')
                one_obj_mask[ceil(BBox_for_ROI["top_left"][0]):ceil(BBox_for_ROI["bottom_right"][0]),
                ceil(BBox_for_ROI["top_left"][1]):ceil(BBox_for_ROI["bottom_right"][1])] = threshed_roi

                # Comment The next 5 lines if training on single class
                with open(os.path.join(img_dir, '_object_settings.json')) as f:
                    object_settings = json.load(f)
                    for n in range(len(object_settings["exported_objects"])):
                        if object_settings["exported_object_classes"][n] == current_class:
                             id = n
                obj = {
                    # "bbox": [np.min(px), np.min(py), np.max(px), np.max(py)],
                    "bbox": [BBox[l]["top_left"][1], BBox[l]["top_left"][0], BBox[l]["bottom_right"][1],
                             BBox[l]["bottom_right"][0]],
                    "bbox_mode": BoxMode.XYXY_ABS,
                    "segmentation": pycocotools.mask.encode(np.asarray(one_obj_mask, order="F")),
                    "category_id": id,  # should be =id if multi class not just obj and 0 otherwise
                }
                objs.append(obj)

        record["annotations"] = objs
        #        record["sem_seg_file_name"] = sem_seg_file_name_and_dir
        dataset_dicts.append(record)
    return dataset_dicts


def parsing():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataview', type=str, default='off', help='view input data (on/ off)')
    parser.add_argument('--train', type=str, default='off', help='start training (on/ off)')
    parser.add_argument('--infer', type=str, default='on', help='perform inference (on by default)')
    parser.add_argument('--num_workers', type=int, default=2, help='Number of workers')
    parser.add_argument('--ims_per_batch', type=int, default=3, help='Number of images in a batch')  # default=2
    parser.add_argument('--batch_size_per_img', type=int, default=1536, help='Batch size per image')
    parser.add_argument('--LR', type=int, default=0.000025, help='Learning rate')  # default=0.00025-
    parser.add_argument('--max_iter', type=int, default=600000, help='Number of iterations')
    parser.add_argument('--decay', type=list, default=[], help='SOLVER.STEPS aka. decay per iter')
    # ------ Multi GPU ------
    #parser.add_argument('--num_gpus', type=int, default=2, help='number of GPUs to run')
    # ------------------------

    return parser.parse_args()


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


def main(args):
    print(args.max_iter)
    json_file_name = 'fat_sample.json'
    #img_dir = r'/media/ghanem/3C448DDD448D99F2/facebook-detectron/data/splitted_FAT_full_rgb'
    #img_dir = r"/media/ghanem/3C448DDD448D99F2/facebook-detectron/data/micro_set"
    start = time.time()

    for d in ["train", "val"]:
        # -----------
        if args.infer == 'on' and d == "train":
            continue
        #object_settings_file = os.path.join(os.path.join(img_dir, d), "_object_settings.json")
        #with open(object_settings_file) as f:
            #object_settings = json.load(f)
            #classes_id_list = []
            #for n in range(len(object_settings["exported_object_classes"])):
                #temp_class_seg_id = object_settings["exported_object_classes"][n]  # ["segmentation_class_id"]
                #classes_id_list.append(temp_class_seg_id)
        # ------------

        DatasetCatalog.register("kitchen0_" + d, lambda d=d: get_obj_dicts(os.path.join(img_dir, d), json_file_name))
        MetadataCatalog.get("kitchen0_" + d).set(
            thing_classes=["object"])  # for classes put = classes_id_list!!!! object_settings["exported_object_classes"]
        '''
        # above is the multi class version
        dataset_dicts = get_obj_dicts(os.path.join(img_dir, d), json_file_name)
        DatasetCatalog.register("kitchen0_" + d, lambda d=d: dataset_dicts)
        MetadataCatalog.get("kitchen0_" + d).set(
            stuff_classes="object")  # if one class stuff_classes="object" if classes =classes_id_list
        '''
        # ----------
    kitchen0_metadata = MetadataCatalog.get("kitchen0_val")
    print("Data dicts loaded:  [1 of 2]")
    print("========================================")

    if args.train == 'on':
        p = "train"

    # --- Config setup ---
    print("Loading cfg")
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))  # WAS 3x.y
    cfg.DATASETS.TRAIN = ("kitchen0_train",)
    cfg.DATASETS.TEST = ()
    cfg.DATALOADER.NUM_WORKERS = args.num_workers
    cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")  # WAS 3x
    # for cfg.MODEL.WEIGHTS, Let training initialize from model zoo
    cfg.SOLVER.IMS_PER_BATCH = args.ims_per_batch
    cfg.SOLVER.BASE_LR = args.LR  # pick a good Learning Rate
    cfg.SOLVER.MAX_ITER = args.max_iter  # 300 iterations seems good for small data
    cfg.SOLVER.STEPS = args.decay  # do not decay learning rate
    cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = args.batch_size_per_img  # faster, and good enough for this small dataset (default: 512)
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # was 21 also for FAT  # only one calss (balloon)
    cfg.MODEL.SEM_SEG_HEAD.NUM_CLASSES = 1
    print("cfg Loaded")
    end = time.time()
    print("setup time: ~", (end - start) / 60, 'minutes')
    if args.dataview == 'on':
        for d in random.sample(dataset_dicts, 15):
            img = cv2.imread(d["file_name"], cv2.IMREAD_UNCHANGED)
            visualizer = Visualizer(img, metadata=kitchen0_metadata, scale=1)
            out = visualizer.draw_dataset_dict(d)
            cv2.imshow("Window", out.get_image())  # cv2.imshow(window_name, image)
            cv2.waitKey(0)

    if args.train == 'on':
        os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)
        print("about to start training")
        trainer = DefaultTrainer(cfg)  # this was from before without the CocoTrainer Class
        trainer.resume_or_load(
            resume=True)  # there are no checkpoints, it will start from iteration 0 and no modified weights
        trainer.train()

    if args.infer == 'on':
        #cfg.OUTPUT_DIR = r"/media/ghanem/3C448DDD448D99F2/facebook-detectron/For_the_paper/training_with_one_class/output"
        #cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")  # "model_final.pth")  # path to the model we just trained
        cfg.MODEL.WEIGHTS = '/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/demo/model_final.pth'
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.95  # Set a custom testing threshold
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        cfg.MODEL.ROI_BOX_HEAD.CLS_AGNOSTIC_BBOX_REG = True
        cfg.MODEL.ROI_MASK_HEAD.CLS_AGNOSTIC_MASK = True
        cfg.MODEL.SEM_SEG_HEAD.NUM_CLASSES = 1

        cfg.MODEL.DEVICE = 'cpu'

        predictor = DefaultPredictor(cfg)

        # Now randomly select several samples to visualize the prediction results.

        # dataset_dicts = get_obj_dicts(img_dir + "/val", json_file_name)
        test_imgs_location = []
        #custom_dir = r"/home/ghanem/Downloads/scenes2/table_YCB_like"
        custom_dir = r"/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/demo/img"
        # custom_dir = r"/media/ghanem/3C448DDD448D99F2/facebook-detectron/data/splitted_FAT_full_rgb/val"
        for root, dirs, files in os.walk(custom_dir):  # put img dir instead of custom_dir
            for file in files:
                #if file.endswith(".jpg") and not file.endswith("seg.png"):
                #if file.endswith(".png") and not file.endswith("seg.png"):
                if file.split("_")[0] == "color":
                    test_imgs_location.append(os.path.join(root, file))

        for img_idx, d in enumerate(random.sample(test_imgs_location, 1)):
            #im = cv2.imread(d["file_name"], cv2.IMREAD_UNCHANGED)
            #im = cv2.imread(d, cv2.IMREAD_UNCHANGED)
            im = cv2.imread(d, cv2.IMREAD_COLOR)
            #im_dim = (960, 540)
            #im = cv2.resize(im, im_dim, interpolation=cv2.INTER_AREA)
            outputs = predictor(
                im)  # format is documented at https://detectron2.readthedocs.io/tutorials/models.html#model-output-format
            v = Visualizer(im,
                           metadata=kitchen0_metadata,
                           scale=1,
                           instance_mode=ColorMode.IMAGE_BW
                           # remove the colors of unsegmented pixels. This option is only available for segmentation models
                           )
            out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
            img = out.get_image()
            img_dir = r"/home/iiwa/segmentation/iiwa_ws/src/klt_dataset_collector/demo/img"
            save_infered_images(save_path=img_dir, folder_name="results", img=img, img_idx=img_idx, enable=True)
            #cv2.imshow("Eval_Rand", img)  # cv2.imshow(window_name, image)
            #cv2.waitKey(0)


if __name__ == '__main__':
    args = parsing()
    main(args)
