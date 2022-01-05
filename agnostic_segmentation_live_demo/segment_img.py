#!/usr/bin/env python
import cv2
import argparse

from agnostic_segmentation_live_demo import agnostic_segmentation

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("img", type=str, help="path to image")
    parser.add_argument("model_path", type=str, help="path to class-agnostic model")
    args = parser.parse_args()
    img = cv2.imread(args.img)
    seg_img = agnostic_segmentation.segment_image(img, img, args.model_path)
    cv2.imshow('Image', seg_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
