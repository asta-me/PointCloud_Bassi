import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt

def is_subarray(A, B):
    for subB in B:
        if (A == subB).all():
            return True
    return False

def compute_edges(geometry):
    geometry.compute_vertex_normals()
    triangles = np.array(geometry.triangles)
    vertices = np.array(geometry.vertices)
    origins = []
    ends = []
    for triangle in triangles:
        for idx in range(3):
            origin = vertices[triangle[idx]]   
            end = vertices[triangle[(idx+1)%3]]
            origins.append(origin)
            ends.append(end)
    selected_origins, selected_ends = remove_dublicates(origins, ends)
    #print('Number of edges in the shape:', selected_origins.shape[0])
    return selected_origins, selected_ends

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
  
def visualizer(function):

    def inner(*args,**kwargs):
        try:
            show_animation = kwargs['show_animation']
        except:
            show_animation = False
        try:
            save_video = kwargs['save_video']
        except:
            save_video = False

        if show_animation or save_video:
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            idx = 0
        for o,e,mesh_list,mesh_to_remove_list in function(*args, **kwargs): 
            if show_animation or save_video:
                for mesh in mesh_list:
                    if idx==0:
                        vis.add_geometry(mesh)
                    else:
                        mesh_to_remove_list
                        vis.update_geometry(mesh)
                for mesh in mesh_to_remove_list:
                    vis.remove_geometry(mesh) 
                vis.poll_events()
                # vis.update_renderer()
                if save_video:
                    full_path = os.path.realpath(__file__) 
                    folder, _ = os.path.split(full_path) # save folder for the video
                    image = vis.capture_screen_float_buffer(False)
                    filename = os.path.join(folder,'video',f"image_{idx:05d}.png")
                    plt.imsave(filename, np.asarray(image), dpi = 1)
                idx+=1     
            yield o,e
        if show_animation or save_video:
            vis.destroy_window()
    return inner
