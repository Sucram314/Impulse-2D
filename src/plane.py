from .body import *
from .linear_algebra import Vector
from .collision import Collision
from math import atan2

class Plane(Body):
    def __init__(self,
                 pos : Vector,
                 norm : Vector,
                 e : float = 0.8,
                 mu_s : float = 0.5,
                 mu_d : float = 0.4):
        
        self.norm = norm
        self.norm.normalize()
        
        ang = atan2(self.norm.y, self.norm.x)

        super().__init__(PLANE, pos, ang, 0, 0, Vector(0,0), 0, e, mu_s, mu_d)

    def bound(self):
        pass

    def collide(self, other : Body):
        if other.kind == PLANE:
            other.collide(self)
        elif other.kind == CIRCLE:
            dpos = other.pos - self.pos

            dist = dpos * self.norm

            if dist < other.rad:
                depth = other.rad - dist
                contact1 = other.pos - self.norm * dist
                contact2 = other.pos - self.norm * other.rad

                return Collision(self, other, self.norm, depth, [contact1, contact2])
            
        elif other.kind == POLYGON:
            min_dist = 0
            contacts = []

            for point in other.transformed_points:
                dpos = point - self.pos

                dist = dpos * self.norm

                if dist < min_dist:
                    min_dist = dist
                
                if dist < 0:
                    contacts.append(point)

            if min_dist < 0:
                depth = -min_dist

                return Collision(self, other, self.norm, depth, contacts)