import open3d as o3d
import numpy as np
from numpy.linalg import norm
from my_open3d_utils import visualizer
from tqdm import tqdm

class Body:
    def __init__(self, radius =1.0, mass = 1.0,
                 pos = np.array((0.,0.,0.)),
                 vel = np.array((0.,0.,0.)),
                 spin_angle=((0.,0.,0.))):
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
        other.mesh.clear() # makes the other body inactive

    def is_body_active(self):
        return np.asarray(self.mesh.vertices).size > 0
    
def get_mesh_from_bodies_list(bodies):
    mesh_list = []
    for body in bodies:
        mesh_list.append(body.mesh)
    return mesh_list


@visualizer
def your_content(time_points, **kwargs):
    """
    Template for adding new contents.
    Create e list of meshes (open3d.geometry.TriangleMesh) to display in each time_point.
        time_points: int, number of geometries to generate/visualize.
        kwargs: boleans, used by the visualizer decorator: show_animation, remove_hidden_lines, show_mesh, save_video. 
    --> mesh_list: list, yielded at each step of the for loop
    """
    
    for i in range(time_points):
        mesh_list = []
        yield mesh_list


@visualizer
def satellite(time_points, **kwargs):
    
    radius_earth = 0.25
    radius_moon = 0.1
    G = 1 # gravitational constant
    M = 5.0 # earth mass
    m = 0.5 # moon mass
    N = 600 # number of frames in the movie
    pos_moon = np.array((0.8, -0.9, 0.0)) # initial position of the moon
    vel_moon = np.array((0.0, -0.2, 1.6)) # initial velocity of the moon
    pos_earth = np.array((0.0, 0.0, 0.0)) # initial position of the earth
    vel_earth = -vel_moon*m/M # initial velocity of the earth with conservation of momentum

    earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_earth) 
    moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_moon) 
    moon.translate(pos_moon)

    dt = 0.01

    for i in range(time_points):

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

        mesh_list = [earth,moon]
        yield mesh_list


@visualizer
def Nbody_problem(time_points, **kwargs):
    
    G = 0.06 # gravitational constant
    Nbodies = 5 # numer of bodies
    dt = 0.02

    # create a list of bodies (and their mesh)
    bodies = []
    np.random.seed(123)
    for index in range(Nbodies):
        pos = 1*np.random.uniform(low=-1.0, high=1.0, size=3)
        vel = 0.45* np.random.uniform(low=-1.0, high=1.0, size=3)
        spin_angle = 2*np.random.uniform(low=-1.0, high=1.0, size=3) *np.pi/180
        body = Body(radius = 0.07, mass= 1.0, pos=pos, vel=vel, spin_angle=spin_angle)
        bodies.append(body) 

    for i in range(time_points):
        for body in bodies:
            acc = np.array((0.,0.,0.))
            for other_body in bodies:
                if body.is_body_active() and other_body.is_body_active() and body is not other_body:
                    distance = body.pos - other_body.pos
                    if norm(distance) < (body.radius+other_body.radius):
                        body.collapse(other_body)
                    else:
                        force = - G*body.mass*other_body.mass * distance/norm(distance)**3 # gravitational force on the body
                        acc += force/body.mass
            body.acc = acc  

        for body in bodies:
            body.move(dt)
            body.spin()

        yield get_mesh_from_bodies_list(bodies)

@visualizer
def vibrations(time_points, **kwargs):
    
    K = 200 # elastic constant
    L = 0.95 # lenght at rest
    dt = 0.01
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

        yield get_mesh_from_bodies_list(bodies)

@visualizer
def escaping_planet(time_points, **kwargs):
    
    radius_earth = 0.25
    radius_moon = 0.1
    G = 1 # gravitational constant
    M = 5.0 # earth mass
    m = 0.5 # moon mass
    pos_moon = np.array((0.8, -0.8, 0.0)) # initial position of the moon
    # vel_moon = np.array((0.0, -0.2, -3.0)) # initial velocity of the moon
    vel_moon = np.array((0.0, -0.2, 3.0)) # initial velocity of the moon
    pos_earth = np.array((0.0, 0.0, 0.0)) # initial position of the earth
    vel_earth = -vel_moon*m/M # initial velocity of the earth with conservation of momentum
    # vel_earth+=np.array((0.0, 0.0, -1))
    vel_earth+=np.array((0.0, 0.0, 1))

    earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_earth) 
    moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_moon) 
    moon.translate(pos_moon)

    dt = 0.01
    for i in tqdm(range(time_points)):
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
        
        rotation_angle_earth = 2*np.pi/time_points
        rotation = earth.get_rotation_matrix_from_axis_angle((0, rotation_angle_earth,0))
        earth.rotate(rotation, center=pos_earth)
        earth.translate(dr_earth)
        
        rotation_angle_moon = 2*2*np.pi/time_points
        revolution = moon.get_rotation_matrix_from_axis_angle((0,rotation_angle_moon,0))
        moon.rotate(revolution, center=pos_moon)
        moon.translate(dr_moon)

        mesh_list = [moon,earth]

        yield mesh_list


@visualizer
def oscillator(time_points, **kwargs):
    
    K = 10 # elastic constant
    L = 1.5 # lenght at rest
    dt = 0.01

    radius = 0.07
    mass = 1
    np.random.seed(1)
    m1 = Body(radius, mass, pos=np.array((0.0,0.2,1.0)), vel=np.array((0.0,0.0,0.0)))
    m2 = Body(radius, mass, pos=np.array(((0.0,-0.2,-1.0))), vel=np.array((0.0,0.0,-0.0)))

    bodies = [m1,m2]
    for body in bodies:
        spinning_angle = 0.01*np.random.uniform(low=-1.0, high=1.0, size=3)
        body.spin_angle = spinning_angle

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
 
        yield get_mesh_from_bodies_list(bodies)
