import open3d as o3d
import numpy as np
from my_open3d_utils import compute_edges

earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.15) 
earth.compute_vertex_normals()
R = earth.get_rotation_matrix_from_xyz((0.35* np.pi, 0, 0.26*np.pi))
earth.rotate(R, center=(0, 0, 0))
triangles = np.array(earth.triangles)
vertices = np.array(earth.vertices)
originsE, endsE = compute_edges(earth)

moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.07) 
moon.compute_vertex_normals()
moon.translate((0.8, 0, -0.8))
R = moon.get_rotation_matrix_from_xyz((0.27*np.pi, 0, 0.25*np.pi))
moon.rotate(R, center=moon.get_center())
triangles = np.array(moon.triangles)
vertices = np.array(moon.vertices)
originsM, endsM = compute_edges(moon)
print('Number of edges in the shape:', originsM.shape[0])

origins = np.concatenate((np.array(originsM),np.array(originsE)),axis=0)
ends = np.concatenate((np.array(endsM),np.array(endsE)),axis=0)

print('Number of edges:', origins.shape)
print('Maximum distance from the center of the field of view:', np.max(origins))

o3d.visualization.draw_geometries([earth,moon])

