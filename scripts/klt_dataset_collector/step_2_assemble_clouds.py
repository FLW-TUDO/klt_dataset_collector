#!/usr/bin/python3
import os
import glob
import numpy as np
import open3d as o3d
import copy
import argparse


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", default='/home/iiwa/segmentation/flw_dataset')
    args = parser.parse_args()

    threshold = 0.01
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    for samples_dir in glob.glob(args.dataset_path+'/*'):
        source = o3d.io.read_point_cloud(glob.glob(samples_dir + '/cloud_0_*.pcd')[0])  # assuming sample 0 is top and to be fixed
        for sample_path in glob.glob(samples_dir + '/*.pcd')[1:]:

            target = o3d.io.read_point_cloud(sample_path)

            draw_registration_result(source, target, np.identity(4))

            print("Initial alignment done!")
            evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
            print(evaluation)

            print("Apply point-to-point ICP")
            reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())
            print(reg_p2p)

            print("Transformation is:")
            print(reg_p2p.transformation)

            #Save PCD
            transformed_pcd = source.transform(reg_p2p.transformation)
            source = transformed_pcd + target
            o3d.visualization.draw_geometries([source])

        o3d.io.write_point_cloud(sample_path + "assembled_cloud.pcd", source)

if __name__ == '__main__':
        main()
