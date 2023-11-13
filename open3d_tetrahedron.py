import open3d as o3d
import numpy as np

def is_subarray(A, B):
    for subB in B:
        if (A == subB).all():
            return True
    return False

def get_sides(triangles, vertices):
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
                         
mesh = o3d.geometry.TriangleMesh.create_tetrahedron()
mesh.compute_vertex_normals()
mesh.compute_adjacency_list()
triangles = np.array(mesh.triangles)
vertices = np.array(mesh.vertices)
origins, ends = get_sides(triangles,vertices)
origins, ends = remove_dublicates(origins, ends)
print('Number of edges in the shape:', origins.shape[0])
o3d.visualization.draw_geometries([mesh])