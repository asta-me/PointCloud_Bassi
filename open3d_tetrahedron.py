import open3d as o3d
import numpy as np
from my_open3d_utils import compute_edges
from numpy.linalg import norm
                         
mesh = o3d.geometry.TriangleMesh.create_tetrahedron()
mesh.compute_vertex_normals()
mesh.compute_adjacency_list()
origins, ends = compute_edges(mesh)
print('Number of edges in the shape:', origins.shape[0])
print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))
o3d.visualization.draw_geometries([mesh])