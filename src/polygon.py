from .body import *
from .linear_algebra import Vector, RotationMatrix
from .collision import Collision
import random
import math

INF = float("inf")

def clip(p1 : Vector, p2 : Vector, pos : Vector, norm : Vector):
    dp1 = (p1 - pos) * norm
    dp2 = (p2 - pos) * norm

    if dp1 < 0 and dp2 < 0:
        return None, None
    elif dp1 >= 0 and dp2 >= 0:
        return p1, p2

    dpos = p2 - p1
    dpos.normalize()

    t = (pos - p1) * norm / (dpos * norm)

    clipped = p1 + dpos * t

    if dp1 <= 0:
        return clipped, p2
    
    return p1, clipped

def random_convex(n : int, r : float):
    x = sorted([random.uniform(-r,r) for i in range(n)])
    y = sorted([random.uniform(-r,r) for i in range(n)])

    minx = x[0]
    maxx = x[-1]
    miny = y[0]
    maxy = y[-1]

    xvec = []
    yvec = []

    last1 = minx
    last2 = minx
    for i in range(1,n-1):
        cur = x[i]
        if random.randint(0,1):
            xvec.append(cur-last1)
            last1 = cur
        else:
            xvec.append(last2-cur)
            last2 = cur

    xvec.append(maxx - last1)
    xvec.append(last2 - maxx)

    last1 = miny
    last2 = miny
    for i in range(1,n-1):
        cur = y[i]
        if random.randint(0,1):
            yvec.append(cur-last1)
            last1 = cur
        else:
            yvec.append(last2-cur)
            last2 = cur

    yvec.append(maxy - last1)
    yvec.append(last2 - maxy)

    random.shuffle(yvec)

    vec = [Vector(xvec[i], yvec[i]) for i in range(n)]
    vec.sort(key=lambda v:math.atan2(v.y,v.x))

    cur = Vector(0,0)
    res = []
    for i in range(n):
        cur += vec[i]
        res.append(cur)

    return res

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
        self.edges : list[Vector] = []

        min_x = max_x = self.transformed_points[0].x
        min_y = max_y = self.transformed_points[0].y

        p1 = self.transformed_points[0]
        p2 = self.transformed_points[1]
        self.edges.append((p2 - p1).normalized())

        for i in range(1, self.num_points):
            p1 = self.transformed_points[i]
            p2 = self.transformed_points[(i+1)%self.num_points]

            min_x = min(min_x, p1.x)
            min_y = min(min_y, p1.y)
            max_x = max(max_x, p1.x)
            max_y = max(max_y, p1.y)

            self.edges.append((p2 - p1).normalized())

        self.AABB.update(min_x, min_y, max_x, max_y)

    def project(self, norm : Vector):
        min_dist = INF
        min_idx = 0
        max_dist = -INF
        max_idx = 0

        for i in range(self.num_points):
            p = self.transformed_points[i]

            dist = p * norm

            if dist < min_dist:
                min_dist = dist
                min_idx = i

            if dist > max_dist:
                max_dist = dist
                max_idx = i

        return min_idx, min_dist, max_idx, max_dist

    def collide(self, other : Body):
        if not self.AABB.collide(other.AABB):
            return
        
        if other.kind == PLANE:
            return other.collide(self)
        elif other.kind == CIRCLE:
            min_dist = INF
            norm : Vector = Vector(0,0)

            for i in range(self.num_points):
                p1 = self.transformed_points[i]
                p2 = self.transformed_points[(i+1)%self.num_points]

                edge = self.edges[i]

                dpos = other.pos - p1

                axis = edge.perpendicular()

                dist = dpos * axis
                closest = other.pos - axis * dist

                if (closest - p1) * edge < 0:
                    closest = p1
                elif (closest - p2) * edge > 0:
                    closest = p2

                dpos = other.pos - closest

                if dpos.squared_length() <= other.rad ** 2:
                    axis = dpos.normalized()

                    if axis * (other.pos - self.pos) < 0:
                        axis = -axis

                    dist = dpos * axis
                    
                    if dist < min_dist:
                        min_dist = dist
                        norm = axis

            if min_dist == INF:
                return
            
            depth = other.rad - min_dist
                                    
            contact1 = other.pos - norm * (other.rad - depth)
            contact2 = other.pos - norm * other.rad

            return Collision(self, other, norm, depth, [contact1, contact2])

        elif other.kind == POLYGON:
            depth = INF
            norm : Vector = Vector(0,0)
            support_idx_A = 0
            support_idx_B = 0

            for edge in self.edges + other.edges:
                axis = edge.perpendicular()
                min_idx_A, min_dist_A, max_idx_A, max_dist_A = self.project(axis)
                min_idx_B, min_dist_B, max_idx_B, max_dist_B = other.project(axis)

                if max_dist_A < min_dist_B or min_dist_A > max_dist_B:
                    return
                
                if max_dist_A < max_dist_B:
                    overlap = max_dist_A - min_dist_B
                    idx_A = max_idx_A
                    idx_B = min_idx_B
                else:
                    overlap = max_dist_B - min_dist_A
                    idx_A = min_idx_A
                    idx_B = max_idx_B

                if overlap < depth:
                    depth = overlap
                    norm = axis
                    support_idx_A = idx_A
                    support_idx_B = idx_B

            dpos = other.pos - self.pos
            if norm * dpos < 0:
                norm = -norm

            # get more perpendicular edge out of the two stemming from the support vertex

            edge1_A = self.edges[support_idx_A]
            edge2_A = self.edges[(support_idx_A-1)%self.num_points]
            dp1_A = abs(edge1_A * norm)
            dp2_A = abs(edge2_A * norm)

            if dp1_A > dp2_A:
                support_idx_A = (support_idx_A-1)%self.num_points

                edge_A = edge2_A
                dp_A = dp2_A
            else:
                edge_A = edge1_A
                dp_A = dp1_A

            p1_A = self.transformed_points[support_idx_A]
            p2_A = self.transformed_points[(support_idx_A+1)%self.num_points]

            edge1_B = other.edges[support_idx_B]
            edge2_B = other.edges[(support_idx_B-1)%other.num_points]
            dp1_B = abs(edge1_B * norm)
            dp2_B = abs(edge2_B * norm)

            if dp1_B > dp2_B:
                support_idx_B = (support_idx_B-1)%other.num_points

                edge_B = edge2_B
                dp_B = dp2_B
            else:
                edge_B = edge1_B
                dp_B = dp1_B

            p1_B = other.transformed_points[support_idx_B]
            p2_B = other.transformed_points[(support_idx_B+1)%other.num_points]

            # clip edges

            if dp_A < dp_B:
                ref_edge = edge_A.normalized()
                ref_p1 = p1_A
                ref_p2 = p2_A

                inc_p1 = p1_B
                inc_p2 = p2_B
                flip = 0
            else:
                ref_edge = edge_B.normalized()
                ref_p1 = p1_B
                ref_p2 = p2_B

                inc_p1 = p1_A
                inc_p2 = p2_A
                flip = 1
                norm = -norm

            inc_p1, inc_p2 = clip(inc_p1, inc_p2, ref_p1, ref_edge)
            if inc_p1 is None: return

            inc_p1, inc_p2 = clip(inc_p1, inc_p2, ref_p2, -ref_edge)
            if inc_p1 is None: return

            contacts = []

            if (inc_p1 - ref_p1) * norm < 0:
                contacts.append(inc_p1)

            if (inc_p2 - ref_p2) * norm < 0:
                contacts.append(inc_p2)

            if contacts:
                if flip:
                    norm = -norm
            
                return Collision(self, other, norm, depth, contacts)