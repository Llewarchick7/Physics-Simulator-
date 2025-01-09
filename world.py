import numpy as np

class World:
    def __init__(self, boundary_min, boundary_max, particles=[], forces=[]):
        self.boundary_min = np.array(boundary_min, dtype=float)  
        self.boundary_max = np.array(boundary_max, dtype=float)  
        self.particles = particles
        self.forces = forces
        
    def add_particles(self, particle):
        self.particles.append(particle)
        
    def add_forces(self, force):
        self.forces.append(force)

    def get_boundary_min(self):
        return self.boundary_min
    
    def get_boundary_max(self):
        return self.boundary_max
    
    def get_particles(self):
        return self.particles
    
    def get_forces(self):
        return self.forces 


