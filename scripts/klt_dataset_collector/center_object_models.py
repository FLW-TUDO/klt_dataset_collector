import open3d as o3d
import copy
import numpy as np

# setup visualizer
viewer = o3d.visualization.Visualizer()
viewer.create_window()
opt = viewer.get_render_option()
opt.point_size = 1
opt.show_coordinate_frame = True

cloud_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset/objects/cereal_box.pcd'
original_pcd = o3d.io.read_point_cloud(cloud_path)
#cl, ind = original_pcd.remove_radius_outlier(nb_points=30, radius=0.01)
#original_pcd = original_pcd.select_by_index(ind)

# remove outliers and down sample to get a better OBB
pcd_down = copy.deepcopy(original_pcd)
#cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
#pcd_down = pcd_down.select_by_index(ind)
pcd_down = pcd_down.voxel_down_sample(voxel_size=0.005)
viewer.add_geometry(original_pcd)
OBB_original = original_pcd.get_oriented_bounding_box()
viewer.add_geometry(OBB_original)

pcd_aligned = copy.deepcopy(original_pcd)
pcd_aligned.translate(np.array([0,0,0]), relative=False)
pcd_aligned.rotate(np.linalg.inv(OBB_original.R))

# rotate object to have main face facing front
rot_mat = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, 0])
pcd_aligned.rotate(rot_mat)

# calculate OBB after aligning
pcd_aligned_down = copy.deepcopy(pcd_aligned)
pcd_aligned_down = pcd_aligned_down.voxel_down_sample(voxel_size=0.02)
OBB_aligned = pcd_aligned_down.get_oriented_bounding_box()
viewer.add_geometry(OBB_aligned)
viewer.add_geometry(pcd_aligned)

# save aligned cloud
o3d.io.write_point_cloud(cloud_path[:-4] + '_aligned.pcd', pcd_aligned)

viewer.run()
viewer.destroy_window()