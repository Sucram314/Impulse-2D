from .body import *
from .linear_algebra import Vector, RotationMatrix
from .collision import Collision

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
        self.edges.append(p2 - p1)

        for i in range(1, self.num_points):
            p1 = self.transformed_points[i]
            p2 = self.transformed_points[(i+1)%self.num_points]

            min_x = min(min_x, p1.x)
            min_y = min(min_y, p1.y)
            max_x = max(max_x, p1.x)
            max_y = max(max_y, p1.y)

            self.edges.append(p2 - p1)

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
            other.collide(self)
        elif other.kind == CIRCLE:
            pass
        elif other.kind == POLYGON:
            depth = float("inf")
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

            norm.normalize()

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
            


            


