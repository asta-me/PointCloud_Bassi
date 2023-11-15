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

    def elastic_collision(self,other):
        vrel = self.vel - other.vel
        rrel = self.pos-other.pos
        distance = norm(rrel)
        ratio0 = 2 * self.mass / (self.mass + other.mass) 
        ratio1 = 2 * self.mass / (self.mass + other.mass) 
        self.vel += - ratio0 * np.dot(vrel,rrel) / distance**2 *rrel 
        other.vel += - ratio1 * np.dot(-vrel,-rrel) / distance**2 *(-rrel)

    def completely_inelastic_collision(self,other):
        m0 = self.mass
        m1 = other.mass
        p = m0 *self.vel + m1 *other.vel
        vm =  p /(m0+m1)
        self.vel = vm
        other.vel = vm

    def set_radius(self,r0,r1):
        # Draw a sphere with a radius proportional to the cubic root of the mass
        volume0 = 4/3*np.pi*r0**3
        volume1 = 4/3*np.pi*r1**3
        r = (3/(4*np.pi)*volume0+volume1)**(1/3) 
        return r

    def collapse(self,other):
        self.completely_inelastic_collision(other)
        new_radius = self.set_radius(self.radius, other.radius)
        print(new_radius/self.radius)
        self.mesh.scale(new_radius/self.radius, center=self.pos)
        self.radius = new_radius
        self.mass += other.mass 
        other.active = False


        
G = 0.06 # gravitational constant
N = 5 # numer of bodies
dt = 0.02

vis = o3d.visualization.Visualizer()
vis.create_window()

# create a list of bodies (and their mesh)
bodies = []
np.random.seed(123)
for index in range(N):
    pos = 1*np.random.uniform(low=-1.0, high=1.0, size=3)
    vel = 0.45* np.random.uniform(low=-1.0, high=1.0, size=3)
    spin_angle = 2*np.random.uniform(low=-1.0, high=1.0, size=3) *np.pi/180
    body = Body(radius = 0.07, mass= 1.0, pos=pos, vel=vel, spin_angle=spin_angle)
    vis.add_geometry(body.mesh)
    bodies.append(body) 

save_video = False
full_path = os.path.realpath(__file__) 
folder, _ = os.path.split(full_path) # save folder for the video 

time_points = 1400
for i in range(time_points):
    
    for body in bodies:
        acc = np.array((0.,0.,0.))
        for other_body in bodies:
            if body.active and other_body.active and body is not other_body:
                distance = body.pos - other_body.pos
                if norm(distance) < (body.radius+other_body.radius):
                    body.collapse(other_body)  
                    vis.remove_geometry(other_body.mesh)   
                else:
                    force = - G*body.mass*other_body.mass * distance/norm(distance)**3 # gravitational force on the body
                    acc += force/body.mass
        body.acc = acc
    

    for body in bodies:
        body.move(dt)
        body.spin()
        if body.active:
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
    
    # print(f'Calculating {origins.shape[0]} edges for rendering {counts} bodies')
    print('Maximum distance from the center of the field of view:', np.max(origins)) # maximun value of x,y or z
             
vis.destroy_window()