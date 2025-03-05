from abc import ABC, abstractmethod
from .linear_algebra import Vector
from .AABB import AABB

PLANE = 0
CIRCLE = 1
POLYGON = 2

class Body(ABC):
    def __init__(self, 
                 kind : int,
                 pos : Vector, 
                 ang : float, 
                 inv_mass : float, 
                 inv_inertia : float, 
                 vel : Vector = Vector(0,0), 
                 ang_vel : float = 0,
                 e : float = 0.8,
                 mu_s : float = 0.5,
                 mu_d : float = 0.4):
        
        self.kind = kind
        self.pos = pos
        self.ang = ang
        self.inv_mass = inv_mass
        self.inv_inertia = inv_inertia

        self.vel = vel
        self.ang_vel = ang_vel

        self.e = e
        self.mu_s = mu_s
        self.mu_d = mu_d

        self.AABB : AABB = AABB()
        self.bound()

    def vel_at(self, pos : Vector):
        return self.vel + self.ang_vel * pos.perpendicular()
    
    def apply_impulse(self, impulse : Vector, contact : Vector):
        self.vel += impulse * self.inv_mass
        self.ang_vel += (contact ^ impulse) * self.inv_inertia

    def correct_position(self, push : Vector):
        self.pos += push * self.inv_mass

    def step(self, delta_time, gravity=Vector(0,-9.8)):
        if self.inv_mass == 0: 
            return

        self.vel += gravity * delta_time
        self.pos += self.vel * delta_time
        self.ang += self.ang_vel * delta_time

        self.bound()

    @abstractmethod
    def bound(self):
        pass

    @abstractmethod
    def collide(self, other):
        pass