import numpy as np
from physics.objects import Particle

class Force:
    def __init__(self, magnitude=0, vector=np.array([0, 0, 0])):
        """ Args:
            magnitude (float, optional): Defaults to 0.
            vector (np.array, optional): Defaults to np.array([0, 0, 0]).
        """
        self.magnitude = magnitude 
        self.vector = vector # 'unit vector notation'
        
        
         
    def apply_force(self, particle):
        """ Summary: This function applies given force to a particle and updates the particles 
        acceleration based on Newtons Second Law of Motion... "a = F_net / mass"

        Args:
            particle (Particle): The given particle the force if being applied to

        Raises:
            ValueError: The particle must have a mass for these calculations
        """
        if particle.mass == 0:
            raise ValueError(f"Mass Cannot be 0")
        
        # Acceleration for this frame is added onto accelertion from previous frame
        acc = self.vector / particle.mass
        particle.acceleration += acc


    def apply_impulse(self, particle, duration):
        """ Summary: This function applies given impulse to a particle and updates the particles 
        velocity based on Newtons Second Law of Motion... "a = F_net / mass"

        Args:
            particle (Particle): The given particle the force if being applied to
            duration (int): How long the impulse is applied for

        Raises:
            ValueError: The particle must have a mass for these calculations
        """
        if particle.mass == 0:
            raise ValueError(f"Mass Cannot be 0")
        
        # Calculate the change in velocity (impulse = force * duration) and add this velocity from previous frame
        delta_v = (self.vector * duration) / particle.mass
        particle.velocity += delta_v
    
    
    def normalize_vector (self):
        return self.vector / np.linalg.norm(self.vector)
    
    def get_x_component(self):
        return self.vector[0]
    
    def get_y_component(self):
        return self.vector[1]
    
    def get_z_component(self):
        return self.vector[2]
    
    
    
    
    
    
    
    
    
    
# Example usage of the Force class below...
if __name__ == "__main__":
    
    force = Force(magnitude=10, vector=np.array([3, 4, 5]))
    
    print(f"Magnitude: {force.magnitude}")
    print(f"Vector: {force.vector}")
    print(f"Normalized Vector: {force.normalize_vector()}")
    print(f"X component: {force.get_x_component()}")
    print(f"Y component: {force.get_y_component()}")
    print(f"Z component: {force.get_z_component()}")