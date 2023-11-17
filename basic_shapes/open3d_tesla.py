import open3d as o3d
import numpy as np
import wget
import os
import sys
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from my_open3d_utils import find_lines_from_mesh

filename = "tesla.obj"

mesh = o3d.io.read_triangle_mesh(filename = os.path.join(parent,'pcd_models',filename))
# mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1,resolution=30) 
mesh.compute_vertex_normals()

R = mesh.get_rotation_matrix_from_xyz((0., -np.pi/2, 0.))
mesh.rotate(R, center=(0, 0, 0))

bbox = mesh.get_axis_aligned_bounding_box()
bbox_points = np.asarray(bbox.get_box_points())
bbox.color = (1, 0, 0)
bbox_points[:, 2] = np.clip(bbox_points[:, 2], a_min=0, a_max=None)
bbox_cropped = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bbox_points))
mesh_cropped = mesh.crop(bbox_cropped)
oriented_bounding_box = mesh.get_oriented_bounding_box()
oriented_bounding_box.color = (0, 1, 0)

line_set = find_lines_from_mesh(mesh)

n_pts = 1000
pcd = mesh.sample_points_uniformly(n_pts)
diameter = np.linalg.norm(np.asarray(pcd.get_min_bound()) - np.asarray(pcd.get_max_bound()))
camera = [0, 0, diameter]
radius = diameter * 100
_, pt_map = pcd.hidden_point_removal(camera, radius)
pcd_visible = pcd.select_by_index(pt_map)
o3d.visualization.draw_geometries([line_set,pcd_visible,bbox,oriented_bounding_box],width=1024, height=768, left=50, top=50)
