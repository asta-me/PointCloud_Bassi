from my_open3d_API import satellite, oscillator,escaping_planet, Nbody_problem, vibrations
from my_open3d_utils import find_lines_from_mesh_list
import numpy as np

"""Example code: use any of the imported function as a generator"""

for mesh_list in satellite(time_points=50,
                                show_animation=True,
                                remove_hidden_lines=True,
                                show_mesh=False,
                                save_video=False):

    _line_set = find_lines_from_mesh_list(mesh_list)
    points = np.asarray(_line_set.points)
    lines = np.asarray(_line_set.lines)

    origins = points[lines[:,0]] 
    ends = points[lines[:,1]]

    print(f'This time-point has {points.shape[0]} points and {lines.shape[0]} lines')


