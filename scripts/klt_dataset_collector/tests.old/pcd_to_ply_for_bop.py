import open3d as o3d
import trimesh
import numpy as np
import glob
import os

path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/models'
for file in glob.glob(path + "/*.pcd"):
    print("Processing: " + file)
    pcd = o3d.io.read_point_cloud(os.path.join(path, file))
    pcd.estimate_normals()
    pcd = pcd.voxel_down_sample(voxel_size=0.001)

    # estimate radius for rolling ball
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 1.5 * avg_dist

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector([radius, radius * 2]))

    # create the triangular mesh with the vertices and faces from open3d
    tri_mesh = trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles),
                               vertex_normals=np.asarray(mesh.vertex_normals),vertex_colors=np.array(mesh.vertex_colors))

    trimesh.convex.is_convex(tri_mesh)

    #tri_mesh.export(os.path.join(path, file[:-3] + "ply"))
    result = trimesh.exchange.ply.export_ply(tri_mesh, encoding='ascii',  vertex_normal=True)
    output_file = open(os.path.join(path, file[:-3] + "ply"), "wb+")
    output_file.write(result)
    output_file.close()

