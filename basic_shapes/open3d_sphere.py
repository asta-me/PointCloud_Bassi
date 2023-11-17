import open3d as o3d
import numpy as np
import os 
import sys
from numpy.linalg import norm
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from my_open3d_utils import find_lines_from_mesh


def _remove_hidden_lines(input_line_set):
    """Removes lines further away from the center of mass of each shape"""
    points = np.asarray(input_line_set.points)
    lines = np.asarray(input_line_set.lines)
    origins = points[lines[:,0]] 
    ends = points[lines[:,1]]
    center = np.mean(origins, axis=0)
    idx_visible = origins[:,2]>center[2]
    visible_origins = origins[idx_visible]
    visible_ends = ends[idx_visible]
    pcd0 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(visible_origins))
    pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(visible_ends))
    t= tuple(range(0,visible_origins.shape[0]))
    correpondences = [(i, i) for i in range(visible_origins.shape[0])]
    line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(pcd0,pcd1,correpondences)
    return line_set

def remove_hidden_lines(input_line_set):
    """"Removes lines using builtin open3D function hidden_point_removal"""
    points = np.asarray(input_line_set.points)
    lines = np.asarray(input_line_set.lines)
    origins = points[lines[:,0]] 
    ends = points[lines[:,1]]
    pcdO = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(origins))
    pcdE = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ends))
    diameter = norm(np.asarray(pcdO.get_min_bound()) - np.asarray(pcdO.get_max_bound()))
    camera = [0, 0, 2]
    radius = diameter * 10
    _visible_mesh, _pts_indexes = pcdO.hidden_point_removal(camera, radius)
    _pcdO = pcdO.select_by_index(_pts_indexes)
    _pcdE = pcdE.select_by_index(_pts_indexes)
    correpondences = [(i, i) for i in range(len(_pts_indexes))]
    line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(_pcdO,_pcdE,correpondences)
    return line_set

earth = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution = 10)

earth.compute_vertex_normals()
R = earth.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
earth.rotate(R, center=(0, 0, 0))
mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0, 0, 0])

line_set0 = find_lines_from_mesh(earth)
#print('Number of edges:', origins.shape)
#print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(line_set0.points)

# Defining the camera and radius parameters for the hidden point removal operation.
diameter = norm(np.asarray(pcd.get_min_bound()) - np.asarray(pcd.get_max_bound()))
camera = [0, 0, diameter]
radius = diameter*10
# Performing the hidden point removal operation on the point cloud using the
# camera and radius parameters defined above.
# The output is a list of indexes of points that are visible.
mesh_visible, pt_map = pcd.hidden_point_removal(camera, radius)
mesh_visible.compute_vertex_normals()
pcd_visible = pcd.select_by_index(pt_map)

line_set = o3d.geometry.LineSet.create_from_triangle_mesh(earth)
colors = [[0, 0.7, 0] for i in range(np.array(line_set.points).shape[0])]    
line_set.paint_uniform_color([0,0.7,0])

line_set1 = o3d.geometry.LineSet()
line_set1.points = line_set.points
line_set1.lines = line_set.lines
#o3d.visualization.draw_geometries([line_set,pcd_visible,mesh_visible])
#print(np.asarray(line_set.lines))
#earth.clear()
# print(np.asarray(earth.vertices).size)
line_set2 = remove_hidden_lines(line_set)
o3d.visualization.draw_geometries([line_set2,pcd])