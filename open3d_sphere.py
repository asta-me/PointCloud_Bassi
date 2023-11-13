import open3d as o3d
import numpy as np


#earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=1.0) 
earth = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution = 20)
earth.compute_vertex_normals()
R = earth.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
earth.rotate(R, center=(0, 0, 0))
triangles = np.array(earth.triangles)
vertices = np.array(earth.vertices)

o3d.visualization.draw_geometries([earth])