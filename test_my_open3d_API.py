from my_open3d_API import satellite, oscillator,escaping_planet, Nbody_problem, vibrations
import numpy as np


"""Example code: use any of the imported function as a generator of origins and ends"""

for origins,ends in satellite(time_points=200, show_animation=True, save_video=False):
    
    #print(f'This time point has {origins.shape[0]} edges')
    print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))


