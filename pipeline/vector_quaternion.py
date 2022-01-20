import numpy as np
from geometry_msgs.msg import Pose

import open3d as o3d

# https://answers.ros.org/question/228896/quaternion-of-a-3d-vector/
def pose_from_vector3D(waypoint):
    #http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    pose= Pose()
    pose.position.x = waypoint[0]
    pose.position.y = waypoint[1]
    pose.position.z = waypoint[2]
    #calculating the half-way vector.
    u = [1,0,0]
    norm = np.linalg.norm(waypoint)
    v = np.asarray(waypoint)/norm
    if (np.array_equal(u, v)):
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
    elif (np.array_equal(u, np.negative(v))):
        pose.orientation.w = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1
    else:
        half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
        pose.orientation.w = np.dot(u, half)
        temp = np.cross(u, half)
        pose.orientation.x = temp[0]
        pose.orientation.y = temp[1]
        pose.orientation.z = temp[2]
    norm = np.math.sqrt(pose.orientation.x*pose.orientation.x + pose.orientation.y*pose.orientation.y + \
        pose.orientation.z*pose.orientation.z + pose.orientation.w*pose.orientation.w)
    if norm == 0:
        norm = 1
    pose.orientation.x /= norm
    pose.orientation.y /= norm
    pose.orientation.z /= norm
    pose.orientation.w /= norm
    return pose

#print(pose_from_vector3D([1,1,1]))
#
#pcd = o3d.io.read_point_cloud('/home/iiwa/Downloads/assembled_cloud.pcd')
#pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
##pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
#center = pcd.get_center()
#pcd_tree = o3d.geometry.KDTreeFlann(pcd)
#[k, idx, _] = pcd_tree.search_knn_vector_3d(center, 1)
#pose = pose_from_vector3D(np.asarray(pcd.colors)[idx[0], :])
#print(idx)
#print(pose)
#np.asarray(pcd.colors)[:, :] = [1, 1, 1]
#np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]
#np.asarray(pcd.normals)[idx[1:], :] *= 100
#pcd.normals = o3d.utility.Vector3dVector())
#o3d.visualization.draw_geometries([pcd])
