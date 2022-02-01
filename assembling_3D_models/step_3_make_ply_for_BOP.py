import open3d as o3d
import open3d.visualization
import trimesh
import numpy as np
import glob
import os

#path = os.getcwd()
path = '/home/gouda/segmentation/3d_scan/scans_15decs/FINALIZING_MODELS/poisson'
for file in glob.glob(os.path.join(path,"*.ply")):
    print("Processing: " + file)
    pcd = o3d.io.read_point_cloud(os.path.join(path, file))

    #open3d.visualization.draw_geometries([pcd])
    pcd.estimate_normals()
    #if file == "****.pcd": # around 1 million points. algorithm gets stuck. reduce by downsample to reduce points
    pcd = pcd.voxel_down_sample(voxel_size=0.0005)
    #    print("red bowl")

    # convert from meter to mm
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * 1000)

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
    output_file = open(os.path.join(path, file[:-4] + "_bop.ply"), "wb+")
    output_file.write(result)
    output_file.close()

