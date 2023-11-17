import open3d as o3d
import numpy as np
from numpy.linalg import norm
import os
import sys
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from my_open3d_utils import compute_edges

                         
mesh = o3d.geometry.TriangleMesh.create_tetrahedron()
mesh.compute_vertex_normals()
mesh.compute_adjacency_list()
origins, ends = compute_edges(mesh)
print('Number of edges in the shape:', origins.shape[0])
print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))
o3d.visualization.draw_geometries([mesh])