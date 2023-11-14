import open3d as o3d
import numpy as np

def is_subarray(A, B):
    for subB in B:
        if (A == subB).all():
            return True
    return False

def get_edges(triangles, vertices):
    origins = []
    ends = []
    for triangle in triangles:
        for idx in range(3):
            origin = vertices[triangle[idx]]   
            end = vertices[triangle[(idx+1)%3]]
            origins.append(origin)
            ends.append(end)
    return(origins, ends)

def remove_dublicates(origins, ends):
    concatenated = np.concatenate([origins,ends],axis=1)
    concatenated_inverted = np.concatenate([ends,origins],axis=1)
    duplicate_indexes = []
    for idx,element in enumerate(concatenated):
        if is_subarray(element, concatenated[0:idx,:]) or (is_subarray(element, concatenated_inverted[0:idx,:])):
            duplicate_indexes.append(idx)
    selected_origins = np.delete(origins,duplicate_indexes,axis=0)
    selected_ends = np.delete(ends,duplicate_indexes,axis=0)
    return selected_origins, selected_ends
  

earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=1.0) 
# earth = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution = 3)
earth.compute_vertex_normals()
R = earth.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
earth.rotate(R, center=(0, 0, 0))
triangles = np.array(earth.triangles)
vertices = np.array(earth.vertices)
originsE, endsE = get_edges(triangles,vertices)
originsE, endsE = remove_dublicates(originsE, endsE)

moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=0.1) 
#moon = o3d.geometry.TriangleMesh.create_sphere(radius=0.1, resolution = 3)
moon.compute_vertex_normals()
moon.translate((5, 0, 0))
R = moon.get_rotation_matrix_from_xyz((np.pi / 4, 0, np.pi / 2))
moon.rotate(R, center=(1, 0, 0))
triangles = np.array(moon.triangles)
vertices = np.array(moon.vertices)
originsM, endsM = get_edges(triangles,vertices)
originsM, endsM = remove_dublicates(originsM, endsM)
print('Number of edges in the shape:', originsM.shape[0])
o3d.visualization.draw_geometries([earth,moon])

