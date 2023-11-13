# -*- coding: utf-8 -*-
"""
Creates a  

@author: Andrea Bassi
"""

#Import pacchetti
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Object3D:

    def rotate_around_axis(self,point, axis, angle):
        #Ruota un punto intorno a un asse dato un angolo in radianti.
        rot_matrix = np.array([[np.cos(angle) + axis[0]**2 * (1 - np.cos(angle)),
                                axis[0] * axis[1] * (1 - np.cos(angle)) - axis[2] * np.sin(angle),
                                axis[0] * axis[2] * (1 - np.cos(angle)) + axis[1] * np.sin(angle)],
                            [axis[1] * axis[0] * (1 - np.cos(angle)) + axis[2] * np.sin(angle),
                                np.cos(angle) + axis[1]**2 * (1 - np.cos(angle)),
                                axis[1] * axis[2] * (1 - np.cos(angle)) - axis[0] * np.sin(angle)],
                            [axis[2] * axis[0] * (1 - np.cos(angle)) - axis[1] * np.sin(angle),
                                axis[2] * axis[1] * (1 - np.cos(angle)) + axis[0] * np.sin(angle),
                                np.cos(angle) + axis[2]**2 * (1 - np.cos(angle))]])
        return np.dot(rot_matrix, point)

    def compute_cube_edges(self, center, side_length, angles):
        #Calcola le coordinate dei punti estremali degli spigoli di un cubo data la posizione del centro, la lunghezza del lato e gli angoli di rotazione.
        half_side_length = side_length / 2.0
        cube_points = np.array([[half_side_length, half_side_length, half_side_length],
                            [half_side_length, -half_side_length, half_side_length],
                            [-half_side_length, -half_side_length, half_side_length],
                            [-half_side_length, half_side_length, half_side_length],
                            [half_side_length, half_side_length, -half_side_length],
                            [half_side_length, -half_side_length, -half_side_length],
                            [-half_side_length, -half_side_length, -half_side_length],
                            [-half_side_length, half_side_length, -half_side_length]])
    
        for i in range(3):
            axis = np.array([1 if i == j else 0 for j in range(3)])
            for j in range(len(cube_points)):
                cube_points[j] = self.rotate_around_axis(cube_points[j], axis, angles[i])
        
        cube_points += center
        return (cube_points)
    
    edges = np.array([[cube_points[0], cube_points[1]],
                      [cube_points[1], cube_points[2]],
                      [cube_points[2], cube_points[3]],
                      [cube_points[3], cube_points[0]],
                      [cube_points[4], cube_points[5]],
                      [cube_points[5], cube_points[6]],
                      [cube_points[6], cube_points[7]],
                      [cube_points[7], cube_points[4]],
                      [cube_points[0], cube_points[4]],
                      [cube_points[1], cube_points[5]],
                      [cube_points[2], cube_points[6]],
                      [cube_points[3], cube_points[7]]])
    
    x1 = edges[:, 0, 0]; y1 = edges[:, 0, 1]; z1 = edges[:, 0, 2];
    x2 = edges[:, 1, 0]; y2 = edges[:, 1, 1]; z2 = edges[:, 1, 2];
    
    return x1, y1, z1, x2, y2, z2





#%% Dati dei cubi in input
"Esempio 3 Cubi "
cubes = [
    {"center": np.array([0, 0.01, 0.7]), "side_length": 0.003, "angles": np.array([np.pi/4, np.pi/3, np.pi/6])},
    {"center": np.array([0.01, 0, 1]), "side_length": 0.005, "angles": np.array([np.pi/4, np.pi/3, np.pi/6])},
    {"center": np.array([0, -0.01, 1.5]), "side_length": 0.006, "angles": np.array([np.pi/4, np.pi/3, np.pi/6])}
    ]



#%% Calcolo Cubi
x1=np.asarray([]); y1=np.asarray([]); z1=np.asarray([]);
x2=np.asarray([]); y2=np.asarray([]); z2=np.asarray([]);

for cube in cubes:
    center = cube["center"]
    side_length = cube["side_length"]
    angles = cube["angles"]
    a1, b1, c1, a2, b2, c2 = compute_cube_edges(center, side_length, angles)
    x1=np.append(x1,a1);
    y1=np.append(y1,b1);
    z1=np.append(z1,c1);
    x2=np.append(x2,a2);
    y2=np.append(y2,b2);
    z2=np.append(z2,c2);
    del a1, b1, c1, a2, b2, c2


#%% Plot Cubi
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for cube in cubes:
    # Plot del centro del cubo
    ax.scatter(cube["center"][0], cube["center"][1], cube["center"][2], c='r', marker='o')

# Plot degli spigoli del cubo
for i in range(len(x1)):
    ax.plot([x1[i], x2[i]], [y1[i], y2[i]], [z1[i], z2[i]], c='b')

ax.set_xlabel('X') # Etichette degli assi
ax.set_ylabel('Y',labelpad=20)
ax.set_zlabel('Z',labelpad=20)
ax.set_aspect('equalxy')
plt.show() # Mostra il grafico
ax.view_init(elev=110, azim=-90, roll=0)

