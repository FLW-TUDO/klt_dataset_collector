import open3d as o3d

#pcd = o3d.io.read_point_cloud("/home/gouda/tmp/annotation/first_collection/00002/assembled_cloud.pcd")
pcd = o3d.io.read_point_cloud("/home/gouda/tmp/annotation/first_collection/00002/cloud_4_1631824362873465.pcd")
o3d.visualization.draw_geometries([pcd])

cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcd_filtered = pcd.select_by_index(ind)
o3d.visualization.draw_geometries([pcd_filtered])

#pcd_downsampled = pcd.voxel_down_sample(voxel_size=0.001)
#o3d.visualization.draw_geometries([pcd_downsampled])

#pcd_uniform_sample = pcd.uniform_down_sample(every_k_points=2)
#o3d.visualization.draw_geometries([pcd_uniform_sample])
