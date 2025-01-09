import numpy as np

class Particle:
    def __init__(self, position, velocity, acceleration, mass, 
                angular_position=None, angular_velocity=None, angular_acceleration=None, rotational_inertia=None, 
                radius=0.1, time=0):
        """ Constructor that handles both constant and time-dependent acceleration and rotational variables:
        -> If position, velocity, and acceleration are arrays, we assume constant values
        -> If they are functions, we assume time-dependent values
        """        
        # Linear motion variables...
        if callable(position) and callable(velocity) and callable(acceleration):
            # When position, velocity, acceleration are defined as a function of time
            self.position_func = position
            self.position = self.position_func(time)
            self.velocity_func = velocity
            self.velocity = self.velocity_func(time)
            self.acceleration_func = acceleration
            self.acceleration = self.acceleration_func(time)
        else:
            # When acceleration is constant for both linear and angular varients and 
            # position, velocity, acceleration are in array or scalar form for respective type of motion
            self.position = np.array(position, dtype=float)
            self.velocity = np.array(velocity, dtype=float)
            self.acceleration = np.array(acceleration, dtype=float)
        
        # Rotational motion variables...
        if callable(angular_position) and callable(angular_velocity) and callable(angular_acceleration):
            # When angular position, velocity, acceleration are defined as a function of time
            self.angular_position_func = angular_position
            self.angular_position = self.angular_position_func(time)
            self.angular_velocity_func = angular_velocity
            self.angular_velocity = self.angular_velocity_func(time)
            self.angular_acceleration_func = angular_acceleration
            self.angular_acceleration = self.angular_acceleration_func(time)
        else:
            # When angular position, velocity, acceleration are defined in scalar form and acceleration is constant
            self.angular_position = angular_position if angular_position is not None else 0
            self.angular_velocity = angular_velocity if angular_velocity is not None else 0
            self.angular_acceleration = angular_acceleration if angular_acceleration is not None else 0

        self.time = time
        self.mass = mass
        self.rotational_inertia = rotational_inertia if rotational_inertia is not None else 0
        self.radius = radius


    def update(self, dt):
        # Update time 
        self.time += dt
        
        # Update linear properties... time-dependent or constnat-acceleration
        if hasattr(self, 'position_func') and hasattr(self, 'velocity_func') and hasattr(self, 'acceleration_func'):
            self.position = self.position_func(self.time)
            self.velocity = self.velocity_func(self.time)
            self.acceleration = self.acceleration_func(self.time)
       
        else:  # Constant acceleration -> Use Constant acceleration equations from kinematics 
            self.position += (self.velocity * dt) + (0.5 * self.acceleration * dt**2)
            self.velocity += self.acceleration * dt
        
        # Update rotational properties... time-dependent or constnat-acceleration
        if hasattr(self, 'angular_position_func') and hasattr(self, 'angular_velocity_func') and hasattr(self, 'angular_acceleration_func'):
            self.angular_position = self.angular_position_func(self.time)
            self.angular_velocity = self.angular_velocity_func(self.time)
            self.angular_acceleration = self.angular_acceleration_func(self.time)
            
        else: # Constant acceleration -> Use Constant acceleration equations from kinematics
            self.angular_velocity += (self.angular_acceleration * dt) + (0.5 * self.angular_acceleration * dt**2)
            self.angular_position += self.angular_velocity * dt



    """ 
    The following group of methods are algorithims for detecting and handling collisions with boundaries 
    and other particles 
    """
    
    
    def detect_wall_collision_discrete(self, boundary_min, boundary_max): 
        """ Discrete Collision Detection:
            -> Reverses respective component of velocity vector if particle is in contact with or beyond boundary 
            -> Common problem that arises is tunneling
        
        Args:
            boundary_min (np.array): Vector represting the minimum boundary locations of enclosed simulation space for each dimension
            boundary_max (np.array): Vector represting the maximum boundary locations of enclosed simulation space for each dimension
        """
        for i in range(len(self.position)):
            if (self.position[i] - self.radius) <= boundary_min[i] or (self.position[i] + self.radius) >= boundary_max[i]:
                self.velocity[i] *= -1
         
            
            
    def handle_wall_collision_continuous(self, boundary_min, boundary_max, dt):
        """ Continuous Collision Handling:
            -> Attempts to solve tunneling issue 
            -> At any given frame, checks the time it will take particle to collide with boundary wall and if 
               that time is less than the time interval between frames, the dt, it means tunneling will occur
            -> If tunneling will occur, we update position and velocity with time interval it will take to collide,  
               visually bringing particle into contact with boundary, then we reverse the velocity, 
               then update for remaining time left in the conventional dt interval 
            

        Args:
            boundary_min (np.array): Vector represting the minimum boundary locations of enclosed simulation space for each dimension
            boundary_max (np.array): Vector represting the maximum boundary locations of enclosed simulation space for each dimension
            dt (float): Time interval between consecutive frames
        """
        t_wall_collision = self.time_to_wall_collision(boundary_min, boundary_max, dt)

        # If there will be a collision within next time step
        if t_wall_collision is not None and t_wall_collision < dt:
            self.update(t_wall_collision)
            self.detect_wall_collision_discrete(boundary_min, boundary_max)
            self.update(dt - t_wall_collision)
        # No collision -> Update normally
        else:
            self.update(dt)
        
        
    def time_to_wall_collision(self, boundary_min, boundary_max, dt):
        """ Helper function for the handle_wall_collision_continuous() function:
            -> Returns the time frame it will take particle to collide with boundary wall from its current position
        """
        times = []

        for i in range(len(self.position)):
            if self.velocity[i] != 0:
                t_min_boundary = ((boundary_min[i] + self.get_radius()) - self.position[i]) / self.velocity[i]
                t_max_boundary = ((boundary_max[i] - self.get_radius()) - self.position[i]) / self.velocity[i]

                if 0 < t_min_boundary <= dt:
                    times.append(t_min_boundary)
                if 0 < t_max_boundary <= dt:
                    times.append(t_max_boundary)

        return min(times) if times else None
    
    

    def elastic_collision(self, other_particle):
        """ 
        Handle an elastic collision between this particle and another particle.
        Conservation of momentum and kinetic energy principles are applied.
        
        Args:
            other_particle (Particle): The other particle making up the collision
        """
        # Distance between each particles center
        delta_position = self.position - other_particle.position
        distance = np.linalg.norm(delta_position)
        
        
        if distance < self.radius + other_particle.radius:
            # Calculate the normal vector
            normal = delta_position / distance
            
            # Relative velocity along the normal
            delta_velocity = self.velocity - other_particle.velocity
            velocity_along_normal = np.dot(delta_velocity, normal)
            
            # Only proceed if particles are moving towards each other
            if velocity_along_normal > 0:
                return  # Particles are moving apart
            
            # Calculate the impulse scalar
            impulse = (2 * velocity_along_normal) / (self.mass + other_particle.mass)
            
            # Update velocities based on impulse
            self.velocity -= (impulse * other_particle.mass * normal)
            other_particle.velocity += (impulse * self.mass * normal)
    
 

    def handle_particle_collision_no_mass(self, other_particle):
        """ 
        Intention: Use only when the mass of particle is irrelevant to simulation.
                   Reflects each particle along the other particles normal vector.
        Summary: 

        Args:
            other_particle (Particle)
        """
        # Distance between each particles center
        delta_position = self.position - other_particle.position
        distance = np.linalg.norm(delta_position)
        
        if distance <= (self.radius + other_particle.radius):
            # Calculate normal vector 
            normal = delta_position / distance
            # Calculate the normal velocity vectors for each particle
            v1_normal = np.dot(self.velocity, normal) * normal
            v2_normal = np.dot(other_particle.velocity, normal) * normal
            
            # Calculate the difference between particles current velocity and normal 
            v1_tangent = self.velocity - v1_normal
            v2_tangent = other_particle.velocity - v2_normal

            self.velocity = v1_tangent + v2_normal
            other_particle.velocity = v2_tangent + v1_normal
            


    def get_position(self):
        return self.position
    
    def get_velocity(self):
        return self.velocity
    
    def get_acceleration(self):
        return self.acceleration
    
    def get_mass(self):
        return self.mass
    
    def get_radius(self):
        return self.radius
    
    def get_time(self):
        return self.time
    
    def get_translational_kinetic_energy(self):
        return 0.5 * self.mass * np.linalg.norm(self.velocity)**2
    
    def get_gravitational_potential_energy(self, reference_point):
        y = self.position[1] - reference_point[1]
        return - (self.mass * 9.81 * y) 

    def get_angular_position(self):
        return self.angular_position
    
    def get_angular_velocity(self):
        return self.angular_velocity

    def get_angular_acceleration(self):
        return self.angular_acceleration
    
    def get_rotational_inertia(self):
        return self.rotational_inertia

    def get_rotational_kinetic_energy(self):
        return 0.5 * self.rotational_inertia * self.angular_velocity**2






class RigidBody(Particle): 
    def __init__(self, position, velocity, acceleration, mass, 
                angular_position=None, angular_velocity=None, angular_acceleration=None, rotational_inertia=None, 
                radius=0.1, time=0):
        super().__init__(position, velocity, acceleration, mass, 
                         angular_position, angular_velocity, angular_acceleration, rotational_inertia, 
                         radius, time) 








class TransverseWave:
    def __init__(self, amplitude, wavelength, frequency, phase_shift=0, speed=None):
        """
        A base class to represent a transverse wave.
        
        Args:
            amplitude (float): Amplitude of the wave.
            wavelength (float): Wavelength of the wave.
            frequency (float): Frequency of the wave.
            phase_shift (float): Phase shift of the wave (default is 0).
            speed (float): Speed of the wave (optional, if not provided, it will be calculated from wavelength and frequency).
        """
        self.amplitude = amplitude 
        self.wavelength = wavelength
        self.frequency = frequency
        self.phase_shift = phase_shift
        
        # Wave Number and Angular Frequency can now be deduced
        self.wave_number = (2 * np.pi) / self.wavelength
        self.angular_frequency = 2 * np.pi * self.frequency
        
        # If wave speed is not provided, calculate it using v = omega / k
        if speed is None:
            self.speed = self.angular_frequency / self.wave_number
        else:
            self.speed = speed


class TransverseTravelingWave(TransverseWave):
    
    def get_displacement_at(self, x, t):
        """
        Get the displacement of the traveling wave at a specific position 'x' and time 't'.
        
        Args:
            x (float): The position at which to evaluate the wave's displacement.
            t (float): The time at which to evaluate the wave's displacement.
        
        Returns:
            float: The displacement of the wave at position x and time t.
        """
        return self.amplitude * np.sin((self.wave_number * x) - (self.angular_frequency * t) + self.phase_shift)

    def get_displacement_profile(self, x_values, t):
        """
        Get the displacement of the traveling wave over a range of x values at a specific time 't'.
        
        Args:
            x_values (np.array): Array of positions to evaluate the wave at.
            t (float): The time at which to evaluate the wave's displacement.
        
        Returns:
            dict: A dictionary where each key is an x position and the value is the displacement at that position at time t.
        """
        displacements = self.amplitude * np.sin((self.wave_number * x_values) - (self.angular_frequency * t) + self.phase_shift)
        return {x: d for x, d in zip(x_values, displacements)}

    def get_velocity_at(self, x, t):
        """
        Get the velocity of the traveling wave at a specific position 'x' and time 't'.
        
        Args:
            x (float): The position at which to evaluate the wave's velocity.
            t (float): The time at which to evaluate the wave's velocity.
        
        Returns:
            float: The velocity of the wave at position x and time t.
        """
        return (-self.angular_frequency * self.amplitude) * np.cos((self.wave_number * x) - (self.angular_frequency * t) + self.phase_shift)

    def get_velocity_profile(self, x_values, t):
        """
        Get the velocity of the traveling wave over a range of x values at a specific time 't'.
        
        Args:
            x_values (np.array): Array of positions to evaluate the wave velocity at.
            t (float): The time at which to evaluate the wave's velocity.
        
        Returns:
            dict: A dictionary where each key is an x position and the value is the velocity at that position at time t.
        """
        velocities = (-self.angular_frequency * self.amplitude) * np.cos((self.wave_number * x_values) - (self.angular_frequency * t) + self.phase_shift)
        return {x: v for x, v in zip(x_values, velocities)}

    @staticmethod
    def interference(wave1, wave2):
        """
        Calculates the interference of two waves and returns a new TransverseTravelingWave object representing the resultant wave.

        Args:
            wave1 (TransverseTravelingWave): The first wave.
            wave2 (TransverseTravelingWave): The second wave.
        
        Returns:
            TransverseTravelingWave: A new wave object representing the resultant wave.
        """
        if wave1.wavelength != wave2.wavelength:
            raise ValueError("Wavelengths must be equal for interference to be calculated")
        if wave1.frequency != wave2.frequency:
            raise ValueError("Frequencies must be equal for interference to be calculated")

        resultant_amplitude = np.sqrt(wave1.amplitude**2 + wave2.amplitude**2 + 2 * wave1.amplitude * wave2.amplitude * np.cos(wave1.phase_shift - wave2.phase_shift))
        resultant_phase_shift = np.arctan2(wave1.amplitude * np.sin(wave1.phase_shift) + wave2.amplitude * np.sin(wave2.phase_shift),
                                           wave1.amplitude * np.cos(wave1.phase_shift) + wave2.amplitude * np.cos(wave2.phase_shift))

        return TransverseTravelingWave(resultant_amplitude, wave1.wavelength, wave1.frequency, resultant_phase_shift)


    @staticmethod
    def resonance(wave1, wave2):
        """
        Calculates the resonance of two waves and returns a new TransverseStandingWave object representing the standing wave.

        Args:
            wave1 (TransverseTravelingWave): The first wave.
            wave2 (TransverseTravelingWave): The second wave.
        
        Returns:
            TransverseStandingWave: A new wave object representing the standing wave.
        """
        # Check preconditions for resonance
        if wave1.amplitude != wave2.amplitude:
            raise ValueError("Amplitudes must be equal for resonance to occur")
        if wave1.wave_number != wave2.wave_number:
            raise ValueError("Wave numbers must be equal for resonance to occur")
        if wave1.angular_frequency != wave2.angular_frequency:
            raise ValueError("Angular frequencies must be equal for resonance to occur")

        # Resultant standing wave
        resultant_amplitude = 2 * wave1.amplitude  # Resultant amplitude is twice the amplitude of the individual waves in resonance
        resultant_wavelength = wave1.wavelength  # Wavelength remains the same
        resultant_frequency = wave1.frequency  # Frequency remains the same

        # Creating a new wave object representing the standing wave
        return TransverseStandingWave(resultant_amplitude, resultant_wavelength, resultant_frequency)



class TransverseStandingWave(TransverseWave):
    
    def get_displacement_at(self, x, t):
        """
        Get the displacement of the standing wave at a specific position 'x' and time 't'.
        
        Args:
            x (float): The position at which to evaluate the wave's displacement.
            t (float): The time at which to evaluate the wave's displacement.
        
        Returns:
            float: The displacement of the wave at position x and time t.
        """
        return 2 * self.amplitude * np.cos(self.wave_number * x) * np.cos(self.angular_frequency * t)

    def get_displacement_profile(self, x_values, t):
        """
        Get the displacement of the standing wave over a range of x values at a specific time 't'.
        
        Args:
            x_values (np.array): Array of positions to evaluate the wave at.
            t (float): The time at which to evaluate the wave's displacement.
        
        Returns:
            dict: A dictionary where each key is an x position and the value is the displacement at that position at time t.
        """
        displacements = 2 * self.amplitude * np.cos(self.wave_number * x_values) * np.cos(self.angular_frequency * t)
        return {x: d for x, d in zip(x_values, displacements)}

    def get_velocity_at(self, x, t):
        """
        Get the velocity of the standing wave at a specific position 'x' and time 't'.
        
        Args:
            x (float): The position at which to evaluate the wave's velocity.
            t (float): The time at which to evaluate the wave's velocity.
        
        Returns:
            float: The velocity of the wave at position x and time t.
        """
        return -2 * self.angular_frequency * self.amplitude * np.cos(self.wave_number * x) * np.sin(self.angular_frequency * t)

    def get_velocity_profile(self, x_values, t):
        """
        Get the velocity of the standing wave over a range of x values at a specific time 't'.
        
        Args:
            x_values (np.array): Array of positions to evaluate the wave velocity at.
            t (float): The time at which to evaluate the wave's velocity.
        
        Returns:
            dict: A dictionary where each key is an x position and the value is the velocity at that position at time t.
        """
        velocities = -2 * self.angular_frequency * self.amplitude * np.cos(self.wave_number * x_values) * np.sin(self.angular_frequency * t)
        return {x: v for x, v in zip(x_values, velocities)}

   
    
        


    
    
    










#class ChargedParticle(Particle):






#class Spring():
