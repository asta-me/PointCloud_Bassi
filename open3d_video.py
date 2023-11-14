import open3d as o3d
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import os
from my_open3d_utils import compute_edges

radius_earth = 1.0
radius_moon = 0.3
pos = np.array((6.0, 1.0, 0.0)) # initial position of the moon
vel = np.array((0.0,-0.3,0.9)) # initial velocity
G = 6 # gravitational constant
M = 1.0 # earth mass
m = 0.1 # moon mass
N = 400 # number of frames in the movie

save_video = False
full_path = os.path.realpath(__file__) 
folder, _ = os.path.split(full_path) # save folder for the video 

earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_earth) 
earth.compute_vertex_normals()
moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_moon) 
moon.translate(pos)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(earth)
vis.add_geometry(moon)

dt = 0.1

for i in range(N):

    acc = - G*M* pos/norm(pos)**3
    vel += acc * dt
    dr = vel * dt
    pos += dr
    print('distance from the earth:', norm(pos))

    rotation_angle = -0.5*2*np.pi/N
    rotation = earth.get_rotation_matrix_from_axis_angle((0, rotation_angle,0))
    earth.rotate(rotation, center=(0, 0, 0))
    
    rotation_angle_moon = 2*2*np.pi/N
    revolution = moon.get_rotation_matrix_from_axis_angle((0,rotation_angle_moon,0))
    moon.rotate(revolution, center=pos)
    moon.translate(dr)
    
    vis.update_geometry(earth)
    vis.update_geometry(moon)
    vis.poll_events()
    vis.update_renderer()

    if save_video:
        image = vis.capture_screen_float_buffer(False)
        filename = os.path.join(folder,'video',f"image_{i:05d}.png")
        plt.imsave(filename, np.asarray(image), dpi = 0.1)

    oringinsM,endsM = compute_edges(moon) # take these points and use it for the hologram calculation
    oringinsE,endsE = compute_edges(earth)

vis.destroy_window()