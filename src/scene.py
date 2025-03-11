from .linear_algebra import Vector
from .body import *
from .circle import Circle
from .polygon import Polygon, random_convex
from .collision import Collision
from math import sin,cos,pi
import random

class Scene:
    def __init__(self, bodies : list[Body] = [], gravity : Vector = Vector(0,-9.8)):
        self.gravity = gravity
        self.bodies : list[Body] = bodies
        self.collisions : list[Collision] = []

        self.paused = False

    def update(self, delta_time):
        if self.paused: 
            return
        
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
            collision.resolve()

        return self.collisions

    def interact(self, left_click : bool, pos : Vector):
        if left_click:
            if random.randint(0,1):
                rad = random.uniform(0.5,5)
                n = random.randint(3,10)
                self.bodies.append(Polygon(pos, random_convex(n, rad)))
            else:
                self.bodies.append(Circle(pos, random.uniform(0.5,5)))
