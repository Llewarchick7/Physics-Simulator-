"""
Example simulations that may be programmatically acheived from the physics logic implemented in this library
"""

from manim import *
import numpy as np
from physics.objects import Particle, TransverseTravelingWave
from physics.world import World
from physics.forces import Force


class ParticleScene(Scene):
    def construct(self):
        # Create a list of Particle objects with different initial positions and velocities 
        particles = [ 
                    Particle(position=np.array([0, 2, 0]), velocity=np.array([0, -2, 0]), acceleration=np.array([0, 0, 0]), mass=0), 
                    Particle(position=np.array([0, -1, 0]), velocity=np.array([0, 2.2, 0]), acceleration=np.array([0, 0, 0]), mass=0), 
                    Particle(position=np.array([1, -2, 0]), velocity=np.array([-1.3, 0.7, 0]), acceleration=np.array([0, 0, 0]), mass=0), 
                    Particle(position=np.array([-1, 1, 0]), velocity=np.array([1.7, -0.5, 0]), acceleration=np.array([0, 0, 0]), mass=0), 
                    Particle(position=np.array([2, 0, 0]), velocity=np.array([-0.9, 2, 0]), acceleration=np.array([0, 0, 0]), mass=0),
                    Particle(position=np.array([-1, -1, 0]), velocity=np.array([1.2, 2.1, 0]), acceleration=np.array([0, 0, 0]), mass=0),
                    Particle(position=np.array([0, 0, 0]), velocity=np.array([-0.9, 0, 0]), acceleration=np.array([0, 0, 0]), mass=0),
                    Particle(position=np.array([2.5, 0, 0]), velocity=np.array([-0.2, 1, 0]), acceleration=np.array([0, 0, 0]), mass=0)
                    ]
    
        # Create visual representations for each particle 
        particle_dots = []
        colors = ['RED', 'BLUE', 'GREEN', 'PURPLE', 'ORANGE', 'PINK', 'YELLOW', 'WHITE']
        for particle, color in zip(particles, colors):
            dot = Dot(point=particle.get_position(), radius=particle.get_radius()).set_color(color)
            particle_dots.append(dot)


        # Create a World object
        world = World(boundary_min=np.array([-3, -3, 0]), boundary_max=np.array([3, 3, 0])) 
        
        # Create visual representation for boundary 
        boundary = Square(side_length=6)


        # Add particle_dot and boundary to scene 
        self.add(*particle_dots, boundary)
        
            
        def update_particles(mob, dt): 
            for i in range(len(particles)): 
                # Update gicen particle position and Handle wall collision for given particle
                particles[i].handle_wall_collision_continuous(world.get_boundary_min(), world.get_boundary_max(), dt)
                
                # Handle potential particle collision with given particle and every other particle in simulation
                for j in range(len(particles)): 
                    if i != j:
                        particles[i].handle_particle_collision_no_mass(particles[j]) 

                # Update the visual representation of goven particle
                particle_dots[i].move_to(particles[i].get_position()) 
                
            
        # Add updaters to each particle dot 
        for dot in particle_dots: 
            dot.add_updater(update_particles) 
            
        # Run the animation 
        self.wait(10)        
        
          # Optionally, you can remove the updater after the animation
        for particle_dot in particle_dots:
            particle_dot.clear_updaters()   
            
        

class ElasticCollisionScene(Scene):
    def construct(self):
        # Create the floor and wall
        floor = Line(start=[-4, -2, 0], end=[4, -2, 0])
        wall_left = Line(start=[-4, -2, 0], end=[-4, 2, 0])
        wall_right = Line(start=[4, -2, 0], end=[4, 2, 0])
        
        # Add the floor and wall to the scene
        self.add(floor, wall_left, wall_right)

        # Text annotations
        title_text = Text("Elastic Collision: Kinetic Energy Conserved", font_size=24).to_corner(UL)
        mass_text = Text("Red Particle is 100x mass of Blue", font_size=18).to_corner(UR)
        friction_text = Text("Frictionless", font_size=18).next_to(floor, DOWN)
        
        self.add(title_text, mass_text, friction_text)
        
        # Create the small and big particles
        small_particle = Particle(position=np.array([-3, (-2 + 0.2), 0]), velocity=np.array([0, 0, 0]), acceleration=np.array([0, 0, 0]), mass=1, radius=0.2)
        big_particle = Particle(position=np.array([-1.5, (-2 + 0.2), 0]), velocity=np.array([0, 0, 0]), acceleration=np.array([0, 0, 0]), mass=100, radius=0.2)
        
        # Create visual representations for each particle
        small_particle_dot = Dot(point=small_particle.get_position(), radius=small_particle.get_radius()).set_color(BLUE)
        big_particle_dot = Dot(point=big_particle.get_position(), radius=big_particle.get_radius()).set_color(RED)
        
        # Add particles to the scene
        self.add(small_particle_dot, big_particle_dot)
        
        # Apply an initial impulse to the big particle
        impulse_force = Force(vector=np.array([-100, 0, 0]))  
        impulse_duration = 0.1  # Duration for which the impulse is applied (in seconds)
        
        # Use the apply_impulse() method to apply the impulse force
        impulse_force.apply_impulse(big_particle, impulse_duration)
        
        def update_scene(mob, dt):
            # Update position and handle wall collisions
            big_particle.handle_wall_collision_continuous(np.array([-4, -2, 0]), np.array([4, 2, 0]), dt)
            small_particle.handle_wall_collision_continuous(np.array([-4, -2, 0]), np.array([4, 2, 0]), dt)
            
            # Handle particle collisions
            small_particle.elastic_collision(big_particle)
            
            # Update visual positions
            big_particle_dot.move_to(big_particle.get_position())
            small_particle_dot.move_to(small_particle.get_position())
        
        # Add updater to the particles
        small_particle_dot.add_updater(update_scene)
        big_particle_dot.add_updater(update_scene)
        
        # Run the animation
        self.wait(22)
        
        # Optionally, remove the updaters
        small_particle_dot.clear_updaters()
        big_particle_dot.clear_updaters()



class WaveInterferenceScene(Scene):
    def construct(self):
        # Setup graphs
        left_graph = Axes(
            x_range=[-10, 10, 1],
            y_range=[-3, 3, 1],
            axis_config={"color": BLUE}
        ).scale(0.5).to_edge(LEFT)
        
        right_graph = Axes(
            x_range=[-10, 10, 1],
            y_range=[-3, 3, 1],
            axis_config={"color": RED}
        ).scale(0.5).to_edge(RIGHT)
        
        x_label_left = left_graph.get_x_axis_label(Tex("x (position)"))
        y_label_left = left_graph.get_y_axis_label(Tex("y (displacement)"))
        
        self.add(left_graph, right_graph, x_label_left, y_label_left)

        # Text annotations
        constituent_graphs = Text("Constituent waves").to_edge(DL)
        resultant_graph = Text("Resultant wave").to_edge(DR)
        animation = Text("Wave Interference").to_edge(UP)
        
        self.add(constituent_graphs, resultant_graph, animation)

        # Create waves
        wave1 = TransverseTravelingWave(amplitude=1, wavelength=3, frequency=1)
        wave2 = TransverseTravelingWave(amplitude=1, wavelength=3, frequency=1, phase_shift=np.pi/2)
        resultant_wave = TransverseTravelingWave.interference(wave1, wave2)

        # Plot waves at t=0
        wave1_graph = left_graph.plot(lambda x: wave1.get_displacement_at(x, 0), color=BLUE)
        wave2_graph = left_graph.plot(lambda x: wave2.get_displacement_at(x, 0), color=GREEN)
        result_graph_line = right_graph.plot(lambda x: resultant_wave.get_displacement_at(x, 0), color=RED)

        # Add graphs to the scene
        self.add(wave1_graph, wave2_graph, result_graph_line)

        # Animate the drawings of the graph
        self.play(Create(wave1_graph), Create(wave2_graph), Create(result_graph_line), run_time=5)

        self.wait(2)





