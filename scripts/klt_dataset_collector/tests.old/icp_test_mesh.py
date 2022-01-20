#!/usr/bin/pythonlange3
import numpy as np
import open3d as o3d
import copy

def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    #source_temp.paint_uniform_color([0,1,0])
    o3d.visualization.draw_geometries([source_temp, target])


def main():
    threshold = 0.004
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    target = o3d.io.read_triangle_mesh('/home/gouda/segmentation/clutter_annotation_tool/test_clouds/obj_000008.ply')
    source = o3d.io.read_point_cloud('/home/gouda/segmentation/clutter_annotation_tool/test_clouds/cloud_0.pcd')

    #o3d.visualization.draw_geometries([source, target])

    #print("Initial alignment done!")
    #evaluation = o3d.pipelines.registration.evaluate_registration(source_choco, target, threshold, trans_init)
    #print(evaluation)

    print("Apply ICP")

    #reg = o3d.pipelines.registration.registration_icp(source_down, target_down, threshold, trans_init,
    #                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #                                                      o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))
    #print("Point to Point ICP finished.")

    radius = 0.002
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=100))
    #source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=100))
    source.compute_triangle_normals()
    source.compute_vertex_normals()
    reg = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                      o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
    print("Point to Plane ICP finished.")
    # transform ICP output and feed it to color ICP
    source.transform(reg.transformation)
    o3d.visualization.draw_geometries([source, target])

    ## filter points from target cloud that are not related to the object so color ICP would work better
    #target_tree = o3d.geometry.KDTreeFlann(target)
    #target_mask = np.zeros(len(target.points), dtype=bool)
    #source_points_idx = list()
    #for i in range(len(source_down.points)):
    #    point = source_down.points[i]
    #    [k, idx, _] = target_tree.search_radius_vector_3d(point, 0.005)
    #    target_mask[idx[1:]] = True
    #    if k > 1:
    #        source_points_idx.append(i)

    #target_points_idx = np.where(target_mask == True)[0]
    #new_target = o3d.geometry.PointCloud()
    #new_target.points = o3d.utility.Vector3dVector(np.asarray(target.points)[target_points_idx])
    #new_target.colors = o3d.utility.Vector3dVector(np.asarray(target.colors)[target_points_idx])
    #new_target.normals = o3d.utility.Vector3dVector(np.asarray(target.normals)[target_points_idx])
    #source_points_idx = np.array(source_points_idx)
    #new_source = o3d.geometry.PointCloud()
    #new_source.points = o3d.utility.Vector3dVector(np.asarray(source_down.points)[source_points_idx])
    #new_source.colors = o3d.utility.Vector3dVector(np.asarray(source_down.colors)[source_points_idx])
    #new_source.normals = o3d.utility.Vector3dVector(np.asarray(source_down.normals)[source_points_idx])
    #o3d.visualization.draw_geometries([new_source, new_target])

    #reg = o3d.pipelines.registration.registration_colored_icp(
    #    source, target, radius, trans_init,
    #    o3d.pipelines.registration.TransformationEstimationForColoredICP(lambda_geometric=0.1),
    #    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=30))
    #print("color ICP finished.")

    print("Transformation is:")
    print(reg.transformation)
    source.transform(reg.transformation)
    o3d.visualization.draw_geometries([source, target])


if __name__ == '__main__':
        main()
