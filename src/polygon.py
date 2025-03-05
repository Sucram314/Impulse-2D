from .body import *
from .linear_algebra import Vector, RotationMatrix
from .collision import Collision

class Polygon(Body):
    def __init__(self, 
                 pos : Vector,
                 points : list[Vector],
                 ang : float = 0,
                 inv_density : float = 1,
                 vel : Vector = Vector(0,0), 
                 ang_vel : float = 0,
                 e : float = 0.8,
                 mu_s : float = 0.5,
                 mu_d : float = 0.4):
        
        self.num_points = len(points)
        
        area = 0
        centroid = Vector(0,0)
        inertia = 0

        for i in range(self.num_points):
            p1 = points[i]
            p2 = points[(i+1)%self.num_points]

            cross = p1 ^ p2
            area += cross
            centroid += cross * (p1 + p2)
            inertia += cross * (p1 * p1 + p1 * p2 + p2 * p2)

        area /= 2
        centroid /= 6 * area

        area = abs(area)

        points = [point - centroid for point in points]

        for i in range(self.num_points):
            p1 = points[i]
            p2 = points[(i+1)%self.num_points]

            cross = p1 ^ p2
            inertia += cross * (p1 * p1 + p1 * p2 + p2 * p2)

        inertia /= 12
        inertia = abs(inertia)

        inv_mass = inv_density / area
        inv_inertia = 1 / inertia

        self.points = points

        super().__init__(POLYGON, pos, ang, inv_mass, inv_inertia, vel, ang_vel, e, mu_s, mu_d)

    def bound(self):
        self.rotation_matrix = RotationMatrix(self.ang)
        self.transformed_points = [point * self.rotation_matrix + self.pos for point in self.points]

        min_x = max_x = self.transformed_points[0].x
        min_y = max_y = self.transformed_points[0].y

        num_points = len(self.transformed_points)
        for i in range(1, num_points):
            p = self.transformed_points[i]

            min_x = min(min_x, p.x)
            min_y = min(min_y, p.y)
            max_x = max(max_x, p.x)
            max_y = max(max_y, p.y)

        self.AABB.update(min_x, min_y, max_x, max_y)

    def collide(self, other : Body):
        if not self.AABB.collide(other.AABB):
            return
        
        if other.kind == PLANE:
            other.collide(self)
        elif other.kind == CIRCLE:
            pass
        elif other.kind == POLYGON:
            pass