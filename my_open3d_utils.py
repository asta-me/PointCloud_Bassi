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

def find_visible_lines_from_mesh_list(mesh_list):
    line_sets_list = []
    for mesh in mesh_list:
        if np.asarray(mesh.vertices).size > 0:
            line_set = find_lines_from_mesh(mesh)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(line_set.points)
            lines = line_set.lines
            pts = np.array(pcd.points)
            center = np.mean(pts, axis=0)
            idx_visible = pts[:,2]>center[2] 
            pcd_visible = pcd.select_by_index(idx_visible) 
            visible_pts = pts[idx_visible]
            print(visible_pts)
            line_sets_list.append(line_set)
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

