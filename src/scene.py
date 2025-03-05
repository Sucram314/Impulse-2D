from .linear_algebra import Vector
from .body import *
from .circle import Circle
from .polygon import Polygon
from .collision import Collision
from math import sin,cos,pi
import random

class Scene:
    def __init__(self, bodies : list[Body] = [], gravity : Vector = Vector(0,-9.8)):
        self.gravity = gravity
        self.bodies : list[Body] = bodies
        self.collisions : list[Collision] = []

        self.paused = False

    def update(self, delta_time, debug=False):
        if self.paused: 
            return
        
        debug_points = []

        num_bodies = len(self.bodies)

        for body in self.bodies:
            body.step(delta_time, self.gravity)

        self.collisions = []
        for i in range(num_bodies-1):
            A = self.bodies[i]

            for j in range(i+1, num_bodies):
                B = self.bodies[j]

                collision = A.collide(B)

                if collision is not None:
                    self.collisions.append(collision)
    
        for collision in self.collisions:
            if debug: debug_points.extend(collision.contacts)
            collision.resolve()

        return debug_points

    def interact(self, left_click : bool, pos : Vector):
        if left_click:
            rad = random.uniform(0.5,5)
            n = random.randint(3,10)
            self.bodies.append(Polygon(pos, [Vector(rad*cos(i*2*pi/n), -rad*sin(i*2*pi/n)) for i in range(n)]))
