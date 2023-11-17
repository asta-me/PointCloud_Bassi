import open3d as o3d
import wget
import os
import sys
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from my_open3d_utils import compute_edges

full_path = os.path.realpath(__file__) 
folder, _ = os.path.split(full_path)
filename = "tesla.obj"

mesh = o3d.io.read_triangle_mesh(filename = os.path.join(folder,'pcd_models',filename))
mesh.compute_vertex_normals()
#mesh.simplify_quadric_decimation(target_number_of_triangles=100)
#mesh.sample_points_uniformly(100)
print(mesh)
#origins,ends = compute_edges(mesh)
#print(f'This shape has {origins.shape[0]} edges')

o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True,width=1024, height=768, left=50, top=50)
