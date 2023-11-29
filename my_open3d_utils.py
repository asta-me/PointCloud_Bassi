import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt
from numpy.linalg import norm

def find_lines_from_mesh(mesh):
    line_set = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    #colors = [[0, 0.7, 0] for i in range(np.array(line_set.points).shape[0])] 
    return line_set   

def merge_line_sets(line_sets):
    line_set = o3d.geometry.LineSet()
    for l_set in line_sets:
        line_set = line_set+l_set
    return line_set

def find_lines_from_mesh_list(mesh_list):
    line_sets_list = []
    for mesh in mesh_list:
        if np.asarray(mesh.vertices).size > 0:
            _line_set = find_lines_from_mesh(mesh)
            line_sets_list.append(_line_set)
    full_line_set = merge_line_sets(line_sets_list) 
    return full_line_set

def _remove_hidden_lines(input_line_set):
    """Removes lines further away from the center of mass of each shape"""
    points = np.asarray(input_line_set.points)
    lines = np.asarray(input_line_set.lines)
    origins = points[lines[:,0]] 
    ends = points[lines[:,1]]
    center = np.mean(origins, axis=0)
    idx_visible = origins[:,2]>center[2]
    visible_origins = origins[idx_visible]
    visible_ends = ends[idx_visible]
    pcd0 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(visible_origins))
    pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(visible_ends))
    t= tuple(range(0,visible_origins.shape[0]))
    correpondences = [(i, i) for i in range(visible_origins.shape[0])]
    line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(pcd0,pcd1,correpondences)
    return line_set

def remove_hidden_lines(input_line_set):
    """"Removes lines using builtin open3D function hidden_point_removal"""
    points = np.asarray(input_line_set.points)
    lines = np.asarray(input_line_set.lines)
    origins = points[lines[:,0]] 
    ends = points[lines[:,1]]
    pcdO = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(origins))
    pcdE = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ends))
    diameter = norm(np.asarray(pcdO.get_min_bound()) - np.asarray(pcdO.get_max_bound()))
    camera = [0, 0, diameter]
    radius = diameter * 10
    _visible_mesh, _pts_indexes = pcdO.hidden_point_removal(camera, radius)
    _pcdO = pcdO.select_by_index(_pts_indexes)
    _pcdE = pcdE.select_by_index(_pts_indexes)
    correpondences = [(i, i) for i in range(len(_pts_indexes))]
    line_set = o3d.geometry.LineSet.create_from_point_cloud_correspondences(_pcdO,_pcdE,correpondences)
    return line_set

def find_visible_lines_from_mesh_list(mesh_list):
    line_sets_list = []
    for mesh in mesh_list:
        if np.asarray(mesh.vertices).size > 0:
            _line_set = find_lines_from_mesh(mesh)
            line_sets_list.append(remove_hidden_lines(_line_set))
    full_line_set = merge_line_sets(line_sets_list) 
    return full_line_set

def _find_visible_lines_from_mesh_list(mesh_list):
    line_sets_list = []
    for mesh in mesh_list:
        if np.asarray(mesh.vertices).size > 0:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(mesh.vertices)
            diameter = norm(np.asarray(pcd.get_min_bound()) - np.asarray(pcd.get_max_bound()))
            camera = [0, 0, diameter]
            radius = diameter * 10
            _visible_mesh, _pts_indexes = pcd.hidden_point_removal(camera, radius)
            _visible_line_set = find_lines_from_mesh(_visible_mesh)
            line_sets_list.append(_visible_line_set)
    full_line_set = merge_line_sets(line_sets_list) 
    return full_line_set

def crop_meshes(mesh_list):
    """Crops the mesh along z (axis 2) cutting the bounding box."""
    cropped_mesh_list = []
    for mesh in mesh_list:
        #bbox = mesh.get_oriented_bounding_box()
        bbox = mesh.get_axis_aligned_bounding_box()
        bbox_points = np.asarray(bbox.get_box_points())
        bbox_points[:,2] = np.clip(bbox_points[:,2], a_min=0.0, a_max=None)
        bbox_cropped = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bbox_points))
        mesh_cropped = mesh.crop(bbox_cropped)
        cropped_mesh_list.append(mesh_cropped)
    return cropped_mesh_list

def visualizer(function):
    def inner(*args,**kwargs):
        try:
            show_animation = kwargs['show_animation']
        except:
            show_animation = False
        try:
            show_mesh = kwargs['show_mesh']
        except:
            show_mesh = False
        try:
            save_video = kwargs['save_video']
        except:
            save_video = False
        try:
            remove_hidden_lines = kwargs['remove_hidden_lines']
        except:
            remove_hidden_lines = False
        if show_animation or save_video:
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            time_idx = 0
        for mesh_list in function(*args, **kwargs):
            if remove_hidden_lines:
                # cropped_list = crop_meshes(mesh_list)
                #last_line_set = find_lines_from_mesh_list(cropped_list)
                last_line_set = find_visible_lines_from_mesh_list(mesh_list)
            else:
                last_line_set = find_lines_from_mesh_list(mesh_list)
            if show_animation or save_video:
                if time_idx==0:
                    pcd = o3d.geometry.PointCloud(last_line_set.points)
                    line_set = o3d.geometry.LineSet(last_line_set.points,last_line_set.lines)
                    vis.add_geometry(pcd)
                    vis.add_geometry(line_set)
                    if show_mesh:
                        for mesh in mesh_list:
                            mesh.compute_vertex_normals()
                            vis.add_geometry(mesh)
                else:
                    pcd.points = last_line_set.points
                    line_set.points = last_line_set.points 
                    line_set.lines = last_line_set.lines
                pcd.paint_uniform_color([0.7, 0.7, 0])
                line_set.paint_uniform_color([0, 0.7, 0])
                vis.update_geometry(pcd)
                vis.update_geometry(line_set)
                if show_mesh:
                    for mesh in mesh_list:
                        mesh.compute_vertex_normals()
                        vis.update_geometry(mesh)
                vis.poll_events()
                vis.update_renderer()
                if save_video:
                    full_path = os.path.realpath(__file__) 
                    folder, _ = os.path.split(full_path) # save folder for the video
                    image = vis.capture_screen_float_buffer(False)
                    filename = os.path.join(folder,'video',f"image_{time_idx:05d}.png")
                    plt.imsave(filename, np.asarray(image), dpi = 1)
                time_idx+=1     
            yield mesh_list
        if show_animation or save_video:
            vis.destroy_window()
    return inner

"""Added by Marco Astarita"""

from PIL import Image
import time
import sys
if sys.version_info[0] < 3:
    get_time=time.clock
else:
    get_time=time.perf_counter

def max_extent(z,lam,d,res):
    L=d*res;
    theta_max=np.arcsin( lam / (2*d) )
    z_min=L/(2*np.tan(theta_max));
    extent = 2 * (z-z_min)*np.tan(theta_max)
    return extent , z_min

#Prendo origins e ends, muovo l'immagine a distanza z=depth, riscalo x e y 
#in modo che rientrino nell'ampiezza del cono luce a distanza depth, con margine margin.
#Se image extent=0, viene calcolato, altrimenti puoi fissare il valore con cui 
#è stato coreato l'oggetto 3D
def oe_to_input(origins,ends,depth,lam,d,res,margin=0.005,image_extent=0):
    x1=origins[:,0]; y1=origins[:,1]; z1=origins[:,2];
    x2=ends[:,0]; y2=ends[:,1]; z2=ends[:,2];    
    #Riscalo x e y in modo che rientrino (circa)
    
    if image_extent==0:
        image_extent=max(extent_x,extent_y);
        extent_y = np.max([y1,y2]) - np.min([y1,y2]); 
        extent_z = np.max([z1,z2]) - np.min([z1,z2]); 
        extent_x = np.max([x1,x2]) - np.min([x1,x2]);
    
    scale_fact=((max_extent(depth,lam,d,res)[0] - margin ) / image_extent);
    x1*= scale_fact; y1*= scale_fact; z1*= scale_fact;
    x2*= scale_fact; y2*= scale_fact; z2*= scale_fact;
    #Muovo il centro alla distanza focale depth
    z1 += depth;
    z2 += depth;
    return x1,y1,z1,x2,y2,z2

def phase_to_bmp(phase):
    phase_bmp=((phase/2/np.pi + 0.5)*255).astype("uint8");    
    return phase_bmp

def save_frame(frame, i, title, subfolder):
    folder_path = os.path.join(os.getcwd(), subfolder)
    os.makedirs(folder_path, exist_ok=True)
    # Costruisci il percorso completo del file
    file_path = os.path.join(folder_path, f"{title}_{i}.bmp")
    # Salva l'immagine
    Image.fromarray(frame).save(file_path)

def rs(x1, y1, z1, x2, y2, z2, d, lam, res):
    t=get_time()
    
    Lx=d*res;
    PupilRadius =Lx;
    
    #Creation of a list of the SLM pixels contained in the pupil
    slm_xcoord,slm_ycoord=np.meshgrid(np.linspace(-1.0,1.0,res),np.linspace(-1.0,1.0,res))
    #Grid of res equally spaced points between -1.0 and 1.0
    pup_coords=np.where(slm_xcoord**2+slm_ycoord**2<1.0)
    #Mask=0 in the circle inscribed in the SLM square, 0 elsewhere.
    #Given in the form of a Tuple (xvalues,yvalues) off all the points inside the circle.
    #TO VISUALIZE IT: fig, ax = plt.subplots();ax.scatter(pup_coords[1], pup_coords[0], s=1, c='black');plt.show()
    
    #Array containing the phase of the field at each created spot
    pists=np.random.random(x1.shape[0])*2*np.pi
    #1D NumPy array containing random piston values between 0 and 2*pi. 
    #We want to generate each spot with a radom added phase.
    #NB:shape (100,), different from a 2D np array with shape (100,1)
    #NB: x are not physical coordinates on SLM but unitary x coordinates of the points inside the pupil.
    
    #Conversion of the coordinates arrays in microns
    slm_xcoord = slm_xcoord*d*float(res)/2.0
    slm_ycoord = slm_ycoord*d*float(res)/2.0
    #Centered Physical coordinates on SLM.
    #Given L the SLM size, the scaling factor brings from [-1 1] to [-L/2 to L/2]
    
    #Computation of the phase patterns generating each single spot independently
    slm_p_phase = np.zeros((x1.shape[0],pup_coords[0].shape[0]))
    #Initialize zeros array with size [numberofpointstogenerate , numberofpointsinsidethepupil] 
    
    for i in range(x1.shape[0]):
        #For every trap (i index) want to generate
        #Create an array of the values slm_p_phase(i,j), j index of the points inside the pupil.
        #slm_p_phase[i,j] corresponds to the literature Delta_j^i
        #Basically calculates an independent lens solution for every trap.
        
        #Coordinate Conversion
        # Line Parameters Conversion  
        [x_mean, y_mean, z0]= [(x1[i]+x2[i])/2, (y1[i]+y2[i])/2, (z1[i]+z2[i])/2 ]
        
        gamma = np.arctan2( z2[i] - z1[i] ,\
                           np.sqrt( (x2[i] - x1[i])**2 + (y2[i] - y1[i])**2) );
            
        alpha = - np.arctan2(y2[i] - y1[i], x2[i] - x1[i]);
        a_1 = np.linalg.norm(np.asarray([x2[i],y2[i],z2[i]]) - np.asarray( [x1[i],y1[i],z1[i]])); #Segment Length
        a = a_1*np.cos(gamma);         #Projection on SLM Length
        
        # Center in Image Plane Coordinates
        x0 = x_mean*np.cos(alpha) - y_mean*np.sin(alpha);  
        y0 = x_mean*np.sin(alpha) + y_mean*np.cos(alpha);
        
        #Image Plane
        x_rot = slm_xcoord[pup_coords]*np.cos(alpha) - slm_ycoord[pup_coords]*np.sin(alpha);            #Image Plane Coordinates 
        y_rot = slm_xcoord[pup_coords]*np.sin(alpha) + slm_ycoord[pup_coords]*np.cos(alpha);
        R= z0 - (x_rot * np.tan(gamma)) ;  #Variable Line Distance from Hologram Plane
        R2_inv= (PupilRadius-a)/(z0*PupilRadius);
        #Per il secondo raggio di curvatura conviene definire l'inverso, così che non è mai infinito.
        
        # Cylindrical wave + Restriction + Transverse & Axial Tilt + Shifted    
        slm_p_phase[i,:]=\
            (np.pi * y_rot**2 / (lam * R) ) + \
            (2*np.pi * (x0*x_rot + y0*y_rot) / (lam * z0) )+\
            (np.pi * R2_inv* x_rot**2 / lam )
               
        # slm_p_phase[i,:]=\
        #     2.0*np.pi/(lam*(f*10.0**3))*\
        #     (x[i]*slm_xcoord[pup_coords]+y[i]*slm_ycoord[pup_coords])+\
        #     (np.pi*z[i])/(lam*(f*10.0**3)**2)*\
        #     (slm_xcoord[pup_coords]**2+slm_ycoord[pup_coords]**2)
        # slm_p_phase[i,:]=\
        #     2.0*np.pi/(lam*(f*10.0**3))*\
        #     (x[i]*slm_xcoord[pup_coords]+y[i]*slm_ycoord[pup_coords])+\
        #     (np.pi*z[i])/(lam*(f*10.0**3)**2)*\
        #     (slm_xcoord[pup_coords]**2+slm_ycoord[pup_coords]**2)   
            
            
    #Creation of the hologram, as superposition of all the phase patterns with random pistons
    slm_total_field=np.sum(\
                              1.0/(float(pup_coords[0].shape[0]))\
                              *np.exp(1j*(slm_p_phase+pists[:,None]))\
                              ,axis=0)
    #Total intensity on pupil supposed=1: amplitude of each term independent solution=1/Number of points in the pupil.
    #Phase of each independent solution=2Darray(slm_p_phase (i)) + randompiston(piston(i))
       
    #pists[:,None] goes from 1D to 2D np array, to sum it with slm_p_phase
    #puts all the element in the first dimension, none in the second
    slm_total_phase=np.angle(slm_total_field)

    t=get_time()-t

    #Evaluation of the algorithm performance, calculating the expected intensities of all spots
    spot_fields=np.sum(1.0/(float(pup_coords[0].shape[0]))*np.exp(1j*(slm_total_phase[None,:]-slm_p_phase)),axis=1)
    # Questa cosa corrisponde alla trasformata di fourier valutata in k=0. Quale è la componente di onda piana? quella che poiu andrebbe a focalizzarsi nell'ordine 0?;
    
    ints=np.abs(spot_fields)**2

    #reshaping of the hologram in a square array
    out=np.zeros((res,res))
    out=np.random.uniform(-np.pi,np.pi,(res,res))

    #initialize output array
    out[pup_coords]=slm_total_phase
    #replace output values only in pupil region


    #the function returns the hologram, and a list with efficiency, uniformity and variance of the spots, and hologram computation time
    
    return out,[np.sum(ints),1-(np.max(ints)-np.min(ints))/(np.max(ints)+np.min(ints)),np.sqrt(np.var(ints))/np.mean(ints),t]