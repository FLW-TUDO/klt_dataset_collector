#!/usr/bin/python3

import numpy as np
import open3d as o3d
import copy

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    transformed_pcd = source_temp.transform(transformation)
    icp_pcd = transformed_pcd+target_temp
    o3d.visualization.draw_geometries([source_temp, target_temp])

    #Save PCD
    o3d.io.write_point_cloud("choco_box_icp.pcd", icp_pcd)


def main():
	print("ICP Script Started...")

	source = o3d.io.read_point_cloud("./top.pcd")
	target = o3d.io.read_point_cloud("./left.pcd")
	


	threshold = 0.01
	trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
	source.transform(trans_init)

	draw_registration_result(source, target, np.identity(4))
	
	print("Initial alignment done!")
	evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
	print(evaluation)

	print("Apply point-to-point ICP")
	reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())
	print(reg_p2p)
	
	print("Transformation is:")
	print(reg_p2p.transformation)
	draw_registration_result(source, target, reg_p2p.transformation)
	


main() 