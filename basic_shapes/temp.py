import open3d as o3d
import wget
import os
from my_open3d_utils import compute_edges

#url = 'https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/test/bunny.pcd'
#filename = wget.download(url)
full_path = os.path.realpath(__file__) 
folder, _ = os.path.split(full_path)
filename = "tesla.pcd"
#filename = "tesla.obj"

# Reading the 3D model file as a 3D mesh using open3d.
#mesh = o3d.io.read_triangle_mesh(filename = os.path.join(folder,'pcd_models',filename))
pcd = o3d.io.read_point_cloud(filename = os.path.join(folder,'pcd_models',filename))
print(dir(pcd))

downpcd = pcd.voxel_down_sample(voxel_size=0.01)
print(downpcd)


#o3d.visualization.draw_geometries([pcd],width=1024, height=768, left=50, top=50)
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=10))


# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         downpcd, depth=9)

# radii = [0.005, 0.01, 0.02, 0.04]
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
#     downpcd, o3d.utility.DoubleVector(radii))


# alpha = 0.02
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(downpcd, alpha)
# mesh.compute_vertex_normals()


# origins,ends = compute_edges(mesh)
# print(f'This shape has {origins.shape[0]} edges')

o3d.visualization.draw_geometries([downpcd], mesh_show_back_face=True,width=1024, height=768, left=50, top=50)
