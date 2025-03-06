from .body import *
from .linear_algebra import Vector
from .collision import Collision
from math import pi

class Circle(Body):
    def __init__(self,
                 pos : Vector,
                 rad : float,
                 ang : float = 0,
                 inv_density : float = 1,
                 vel : Vector = Vector(0,0), 
                 ang_vel : float = 0,
                 e : float = 0.8,
                 mu_s : float = 0.5,
                 mu_d : float = 0.4):
        
        self.rad = rad
        inv_mass = 1 / self.rad ** 2 * inv_density
        inv_inertia = 4 / (pi * self.rad ** 4)

        super().__init__(CIRCLE, pos, ang, inv_mass, inv_inertia, vel, ang_vel, e, mu_s, mu_d)

    def bound(self):
        self.AABB.update(self.pos.x - self.rad, self.pos.y - self.rad, self.pos.x + self.rad, self.pos.y + self.rad)

    def collide(self, other : Body):
        if not self.AABB.collide(other.AABB):
            return
        
        if other.kind == PLANE:
            return other.collide(self)
        elif other.kind == CIRCLE:
            dpos = other.pos - self.pos

            sqr_dist = dpos.squared_length()
            if sqr_dist < (self.rad + other.rad) ** 2:
                dist = sqr_dist ** 0.5
                norm = dpos / dist
                depth = self.rad + other.rad - dist

                contact = self.pos + norm * self.rad

                return Collision(self, other, norm, depth, [contact])
        elif other.kind == POLYGON:
            return other.collide(self)