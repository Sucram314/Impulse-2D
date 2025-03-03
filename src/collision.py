from .linear_algebra import Vector
from .body import Body

class Collision:
    def __init__(self, A : Body, B : Body, norm : Vector, depth : float, contacts : list[Vector]):
        self.A = A
        self.B = B
        self.norm = norm
        self.depth = depth
        self.contacts = contacts

    def resolve(self, slop=0.00198, percentage=0.25):
        A = self.A
        B = self.B

        inv_num_contacts = 1 / len(self.contacts)

        for contact in self.contacts:
            rel_A = contact - A.pos
            rel_B = contact - B.pos
            rel_vel = B.vel_at(rel_B) - A.vel_at(rel_A)

            contact_vel = rel_vel * self.norm

            if contact_vel > 0:
                continue

            inv_mass_sum = A.inv_mass + B.inv_mass
            inv_mass_sum += (rel_A ^ self.norm)**2 * A.inv_inertia
            inv_mass_sum += (rel_B ^ self.norm)**2 * B.inv_inertia

            e = min(A.e, B.e)
            j = -(1 + e) * contact_vel / inv_mass_sum * inv_num_contacts

            impulse = self.norm * j
            A.apply_impulse(-impulse, rel_A)
            B.apply_impulse( impulse, rel_B)

            rel_vel = B.vel_at(rel_B) - A.vel_at(rel_A)
            tang = rel_vel - self.norm * contact_vel

            if tang.squared_length() == 0: 
                continue

            tang.normalize()

            j_t = -(rel_vel * tang) / inv_mass_sum * inv_num_contacts

            if j_t == 0:
                continue

            mu_s = (A.mu_s + B.mu_s) * 0.5
            mu_d = (A.mu_d + B.mu_d) * 0.5

            if abs(j_t) <= j * mu_s:
                impulse_t = tang * j_t
            else:
                impulse_t = tang * -j * mu_d

            A.apply_impulse(-impulse_t, rel_A)
            B.apply_impulse( impulse_t, rel_B)

        if self.depth > slop:
            step = self.depth * percentage
            push = self.norm * step / (A.inv_mass + B.inv_mass)

            A.correct_position(-push)
            B.correct_position( push)