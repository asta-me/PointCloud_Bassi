import open3d as o3d
import numpy as np
from my_open3d_utils import compute_edges

earth = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution = 10)
earth.compute_vertex_normals()
R = earth.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
earth.rotate(R, center=(0, 0, 0))


origins, ends = compute_edges(earth)
print('Number of edges:', origins.shape)
print('Maximum distance from the center of the field of view:', np.max(origins))
o3d.visualization.draw_geometries([earth])