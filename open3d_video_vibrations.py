import open3d as o3d
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import os
from my_open3d_utils import compute_edges

class Body:
    def __init__(self, radius =1.0, mass = 1.0,
                 pos = np.array((0.,0.,0.)),
                 vel = np.array((0.,0.,0.)),
                 spin_angle=((0.,0.,0.))):
        self.active = True
        self.radius = radius
        self.mass = mass
        self.pos = pos
        self.vel = vel
        self.acc = np.array((0.,0.,0.))
        self.spin_angle = spin_angle
        self.mesh = o3d.geometry.TriangleMesh.create_icosahedron(radius)
        self.mesh.translate(pos)

    def move(self,dt):
        self.vel += self.acc *dt
        dr = self.vel *dt
        self.pos += dr
        self.mesh.translate(dr)
    
    def spin(self):
        rotation = self.mesh.get_rotation_matrix_from_axis_angle(self.spin_angle)
        self.mesh.rotate(rotation, center=self.pos)

        
K = 200 # elastic constant
L = 0.95 # lenght at rest
N = 3 # numer of bodies
dt = 0.01

vis = o3d.visualization.Visualizer()
vis.create_window()

radius = 0.07
mass = 1
angle = 104.95*np.pi/180
np.random.seed(1)
O = Body(4*radius, 27*mass, pos=np.array((0.,0.,-0.5)))
L1 = 1.05
L2 = 1.05
H1 = Body(radius, mass, pos=np.array((L1*np.sin(angle/2),0.,-0.5+L1*np.cos(angle/2))))
H2 = Body(radius, mass, pos=np.array((-L2*np.sin(angle/2),0.,-0.5+L2*np.cos(angle/2))))

bodies = [H1,O,H2]
for body in bodies:
    spinning_angle = 0.01*np.random.uniform(low=-1.0, high=1.0, size=3)
    body.vel = np.array((0.,0.,0.0))
    body.spin_angle = spinning_angle
    vis.add_geometry(body.mesh)

save_video = False
full_path = os.path.realpath(__file__) 
folder, _ = os.path.split(full_path) # save folder for the video 

time_points = 200
for i in range(time_points):
    
    for idx,body in enumerate(bodies):
        acc = np.array((0.,0.,0.))
        if idx == 0:
            connected_bodies = [bodies[idx+1] ]
        elif idx == len(bodies)-1:
            connected_bodies = [bodies[idx-1] ]
        else:
            connected_bodies = [bodies[idx-1] , bodies[idx+1] ]
        
        for other_body in connected_bodies:
            distance = body.pos - other_body.pos
            delta = norm(distance)
            force =  - K * distance/ delta * (delta-L)    # elastic force
            acc += force/body.mass 

        body.acc = acc
    

    for body in bodies:
        body.move(dt)
        body.spin()
        vis.update_geometry(body.mesh)

    vis.poll_events()
    vis.update_renderer()

    if save_video:
        image = vis.capture_screen_float_buffer(False)
        filename = os.path.join(folder,'video',f"image_{i:05d}.png")
        plt.imsave(filename, np.asarray(image), dpi = 0.1)

    counts = 0
    for body in bodies:
        if body.active:
            o,e = compute_edges(body.mesh)
            if counts == 0:
                origins = o
                ends = e
            else:
                origins = np.concatenate((origins,o))
                ends = np.concatenate((ends,e))
            counts+=1
    
    #print(f'Calculating {origins.shape[0]} edges for rendering {counts} bodies')
    print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))
             
vis.destroy_window()