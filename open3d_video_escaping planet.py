import open3d as o3d
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import os
from my_open3d_utils import compute_edges

radius_earth = 0.25
radius_moon = 0.1
G = 1 # gravitational constant
M = 5.0 # earth mass
m = 0.5 # moon mass
N = 600 # number of frames in the movie
pos_moon = np.array((0.8, -0.9, -1.0)) # initial position of the moon
vel_moon = np.array((0.0, -0.2, -3.0)) # initial velocity of the moon
pos_earth = np.array((0.0, 0.0, -1.0)) # initial position of the earth
vel_earth = -vel_moon*m/M # initial velocity of the earth with conservation of momentum
vel_earth+=np.array((0.0, 0.0, -1))

save_video = False
full_path = os.path.realpath(__file__) 
folder, _ = os.path.split(full_path) # save folder for the video 

earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_earth) 
moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_moon) 
moon.translate(pos_moon)

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(earth)
vis.add_geometry(moon)

dt = 0.01

for i in range(N):

    distance = pos_moon-pos_earth
    force = - G*M*m * distance/norm(distance)**3 # force on the moon
    acc_moon = force/m
    acc_earth = -force/M
    
    vel_moon += acc_moon * dt
    vel_earth += acc_earth * dt
    
    dr_moon = vel_moon * dt
    dr_earth = vel_earth * dt
    
    pos_moon += dr_moon
    pos_earth += dr_earth
    
    rotation_angle_earth = 2*np.pi/N
    rotation = earth.get_rotation_matrix_from_axis_angle((0, rotation_angle_earth,0))
    earth.rotate(rotation, center=pos_earth)
    earth.translate(dr_earth)
    
    rotation_angle_moon = 2*2*np.pi/N
    revolution = moon.get_rotation_matrix_from_axis_angle((0,rotation_angle_moon,0))
    moon.rotate(revolution, center=pos_moon)
    moon.translate(dr_moon)
    
    vis.update_geometry(earth)
    vis.update_geometry(moon)
    vis.poll_events()
    vis.update_renderer()

    if save_video:
        image = vis.capture_screen_float_buffer(False)
        filename = os.path.join(folder,'video',f"image_{i:05d}.png")
        plt.imsave(filename, np.asarray(image), dpi = 0.1)

    originsM,endsM = compute_edges(moon) # take these points and use it for the hologram calculation
    originsE,endsE = compute_edges(earth)
    

    origins = np.concatenate((originsM,originsE))
    ends = np.concatenate((endsM,endsE))

    #print(f'Calculating {origins.shape[0]} edges')
    print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))

vis.destroy_window()