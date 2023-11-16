import open3d as o3d
import numpy as np
from numpy.linalg import norm
from my_open3d_utils import compute_edges, visualizer

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

@visualizer
def satellite(time_points, show_animation=False, save_video=False):
    
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

        originsM,endsM = compute_edges(moon) # take these points and use it for the hologram calculation
        originsE,endsE = compute_edges(earth)
        
        origins = np.concatenate((originsM,originsE))
        ends = np.concatenate((endsM,endsE))
        mesh_list = [earth,moon]
        mesh_to_remove_list = []
        yield(origins,ends,mesh_list,mesh_to_remove_list)


@visualizer
def Nbody_problem(time_points, show_animation=False, save_video=False):
    
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
        mesh_to_remove_list = []
        for body in bodies:
            acc = np.array((0.,0.,0.))
            for other_body in bodies:
                if body.active and other_body.active and body is not other_body:
                    distance = body.pos - other_body.pos
                    if norm(distance) < (body.radius+other_body.radius):
                        body.collapse(other_body) 
                        mesh_to_remove_list.append(other_body.mesh) 
                    else:
                        force = - G*body.mass*other_body.mass * distance/norm(distance)**3 # gravitational force on the body
                        acc += force/body.mass
            body.acc = acc
        

        for body in bodies:
            body.move(dt)
            body.spin()

        counts = 0
        mesh_list = []
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
                mesh_list.append(body.mesh)

        # print(f'Calculating {origins.shape[0]} edges for rendering {counts} bodies')
        # print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))
    
        yield(origins,ends,mesh_list,mesh_to_remove_list)

@visualizer
def vibrations(time_points, show_animation=False, save_video=False):
    
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

        counts = 0
        mesh_list = []
        mesh_to_remove_list = []
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
                mesh_list.append(body.mesh)
        
        yield(origins,ends,mesh_list,mesh_to_remove_list)

@visualizer
def escaping_planet(time_points, show_animation=False, save_video=False):
    
    radius_earth = 0.25
    radius_moon = 0.1
    G = 1 # gravitational constant
    M = 5.0 # earth mass
    m = 0.5 # moon mass
    N = 600 # number of frames in the movie
    pos_moon = np.array((0.8, -0.9, -1.0)) # initial position of the moon
    vel_moon = np.array((0.0, -0.2, -3.0)) # initial velocity of the moon
    pos_earth = np.array((0.0, 0.0, -1.0)) # initial position of the earth
    vel_earth = -vel_moon*m/M # initial velocity of the earth with conservation of momentum
    vel_earth+=np.array((0.0, 0.0, -1))

    earth = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_earth) 
    moon = o3d.geometry.TriangleMesh.create_icosahedron(radius=radius_moon) 
    moon.translate(pos_moon)

    dt = 0.01
    for i in range(N):
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

        originsM,endsM = compute_edges(moon) # take these points and use it for the hologram calculation
        originsE,endsE = compute_edges(earth)
        
        origins = np.concatenate((originsM,originsE))
        ends = np.concatenate((endsM,endsE))

        mesh_list = [earth,moon]
        mesh_to_remove_list = []
        
        yield(origins,ends,mesh_list,mesh_to_remove_list)


@visualizer
def oscillator(time_points, show_animation=False, save_video=False):
    
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

        counts = 0
        mesh_list = []
        mesh_to_remove_list = []
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
                mesh_list.append(body.mesh)
        
        yield(origins,ends,mesh_list,mesh_to_remove_list)



"""Example code: use satellite as a generator of origins, ends and meshes"""
if __name__ == '__main__':

    for origins,ends in oscillator(time_points=500, show_animation=True, save_video=False):
        #print(f'This time point has {origins.shape[0]} edges')
        print('Maximum displacement from the center of the field of view:', np.max(np.abs(origins),axis=0))


