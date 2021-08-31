#!/usr/bin/python3
import glob
import numpy as np
import open3d as o3d
import argparse

import time

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-path", default='/home/iiwa/segmentation/flw_dataset')
    args = parser.parse_args()

    threshold = 0.008
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    for samples_dir in glob.glob(args.dataset_path+'/*'):
        start = time.time()
        target = o3d.io.read_point_cloud(glob.glob(samples_dir + '/cloud_0_*.pcd')[0])  # assuming sample 0 is top and to be fixed
        target, ind = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)

        print(samples_dir)
        for sample_path in glob.glob(samples_dir + '/*.pcd'):
            print(sample_path)
            if sample_path[-19:] == 'assembled_cloud.pcd':
                print('ignoring assembled_cloud.pcd; the file will be overwritten')
                continue
            elif 'cloud_0' in sample_path:
                print('skip cloud_0 file as it is used as the target cloud.')
                continue

            source = o3d.io.read_point_cloud(sample_path)
            source, ind = source.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)

            #o3d.visualization.draw_geometries([source, target])

            #print("Initial alignment done!")
            #evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
            #print(evaluation)

            #print("Apply point-to-point ICP")
            target_downsampled = target.voxel_down_sample(voxel_size=0.001)
            source_downsampled = source.voxel_down_sample(voxel_size=0.001)
            #o3d.visualization.draw_geometries([source_downsampled, target_downsampled])
            reg_p2p = o3d.pipelines.registration.registration_icp(source_downsampled, target_downsampled, threshold, trans_init,
                                                                  o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                                  o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
            #print(reg_p2p)

            #print("Transformation is:")
            #print(reg_p2p.transformation)

            #Save PCD
            # TODO should i reverse and transform the target instead of the source
            transformed_pcd = source.transform(reg_p2p.transformation)
            target = transformed_pcd + target
            print('before downsample:' + str(target))
            print('after  downsample:' + str(target))
            #o3d.visualization.draw_geometries([target])

        o3d.io.write_point_cloud(samples_dir + '/assembled_cloud.pcd', target)
        target = target.voxel_down_sample(voxel_size=0.0005)
        #o3d.visualization.draw_geometries([target])
        print("time:" + str(time.time() - start))

if __name__ == '__main__':
        main()
