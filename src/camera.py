from .linear_algebra import Vector

class Camera:
    def __init__(self, pos : Vector = Vector(0,0), zoom=50, width=1280, height=650):
        self.pos = pos
        self.zoom = zoom

        self.vel = Vector(0,0)
        self.zoom_vel = 0

        self.pan_speed = 50
        self.pan_friction = 0.95
        self.zoom_speed = 1
        self.zoom_friction = 0.95

        self.min_zoom = 3
        self.max_zoom = 100

        self.width = width
        self.height = height
        self.offset = Vector(width//2, height//2)

    def update(self, mouse_pos : Vector, world_pos : Vector, pan_vel, shift, scroll, delta_time):
        speed = self.pan_speed / self.zoom * (1 + shift)
        self.vel += pan_vel * speed

        if scroll:
            self.offset = mouse_pos
            self.pos = world_pos
            self.zoom_vel += scroll * self.zoom_speed * self.zoom

        self.vel *= self.pan_friction
        self.zoom_vel *= self.zoom_friction

        self.pos += self.vel * delta_time
        self.zoom += self.zoom_vel * delta_time
        
        if self.zoom < self.min_zoom:
            self.zoom_vel = 0
            self.zoom = self.min_zoom
        elif self.zoom > self.max_zoom:
            self.zoom_vel = 0
            self.zoom = self.max_zoom

    def to_screen_space(self, pos : Vector):
        res = (pos - self.pos) * self.zoom + self.offset
        res.y = self.height - res.y
        return res

    def to_world_space(self, pos : Vector):
        pos.y = self.height - pos.y
        return (pos - self.offset) / self.zoom + self.pos
