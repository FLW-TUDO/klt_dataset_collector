import os.path

import open3d as o3d
import glob

#pcd = o3d.io.read_point_cloud("/home/gouda/tmp/annotation/first_collection/00002/assembled_cloud.pcd")
original_models_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/models'
downsampled_models_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/models_downsampled'

for model in glob.glob(original_models_path + '/*.ply'):
    model_name = os.path.split(model)[-1]
    pcd = o3d.io.read_point_cloud(os.path.join(original_models_path, model_name))
    #o3d.visualization.draw_geometries([pcd])

    #cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    #pcd_filtered = pcd.select_by_index(ind)
    #o3d.visualization.draw_geometries([pcd_filtered])

    pcd_downsampled = pcd.voxel_down_sample(voxel_size=1)
    #o3d.visualization.draw_geometries([pcd_downsampled])

    #pcd_uniform_sample = pcd.uniform_down_sample(every_k_points=2)
    #o3d.visualization.draw_geometries([pcd_uniform_sample])

    o3d.io.write_point_cloud(os.path.join(downsampled_models_path, model_name), pcd_downsampled, write_ascii=True)
