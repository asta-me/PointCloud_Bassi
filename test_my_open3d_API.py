from my_open3d_API import satellite, oscillator, escaping_planet, Nbody_problem, vibrations
from my_open3d_utils import find_lines_from_mesh_list, oe_to_input, rs, phase_to_bmp,save_frame
import numpy as np

"""Example code: use any of the imported function as a generator"""

#SLM anf Hologram Parameters
d=0.25*8e-6;                 #[m] pixelsize
lam=532e-9;             #[m] wavelength
res=1080;

#Hologram initial distance
depth=0.4;
#Frameindex
i=1;


for mesh_list in escaping_planet(time_points=500,
                                show_animation=True,
                                remove_hidden_lines=False,
                                show_mesh=False,
                                save_video=False):

    line_set = find_lines_from_mesh_list(mesh_list)
    points = np.asarray(line_set.points)
    lines = np.asarray(line_set.lines)

    origins = points[lines[:,0]] 
    ends = points[lines[:,1]]

    x1,y1,z1,x2,y2,z2=oe_to_input(origins,ends,depth,lam,d,res,margin=0.005,image_extent=2);
    
    print(f"Computing {i} of {500}")
    # print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))
    print('x max ', np.max((x1,x2)), 'x min ', np.min ((x1,x2)))
    print('y max ', np.max((y1,y2)), 'y min ', np.min ((y1,y2)))
    print('z max ', np.max((z1,z2)), 'z min ', np.min ((z1,z2))) 
    
    # # print(f'This time-point has {points.shape[0]} points and {lines.shape[0]} lines')
    
    rs_phase, performance = rs(x1, y1, z1, x2, y2, z2, d, lam, res)
    rs_phase_bmp=phase_to_bmp(phase=rs_phase);
    save_frame(rs_phase_bmp,i,title="Escaping_planet_test",subfolder="test_Escaping_Planet__04_to_09")    
 
    # Update counter
    i+=1;

print('\n Finished, tutto apposto')
