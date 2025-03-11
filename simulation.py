
import pygame
import pygame.gfxdraw
import sys
from math import sin,cos,pi
from src.linear_algebra import Vector
from src.body import *
from src.collision import Collision
from src.plane import Plane
from src.circle import Circle
from src.polygon import Polygon
from src.scene import Scene
from src.camera import Camera

pygame.font.init()
font = pygame.font.SysFont("Verdana",30)

bodies : list[Body] = [
    Plane(Vector(0,0),Vector(0,1)),
    Circle(Vector(0,2),1), 
    #Circle(Vector(4,2),1,vel=Vector(-2,1)),
    Polygon(Vector(0,2),[Vector(2,-2),Vector(-2,-2),Vector(-2,2),Vector(2,2)], ang_vel=0),
    #Polygon(Vector(-1,13),[Vector(-1,-1),Vector(-1,1),Vector(1,1),Vector(1,-1)], ang_vel=0),
]

class Display:
    def __init__(self,width=1280,height=650,bg=(0,0,0),fps=120):
        self.width = width
        self.height = height
        self.hwidth = width//2
        self.hheight = height//2
        self.bg = bg
        self.fps = fps
        
        self.screen = pygame.display.set_mode((width,height))
        self.corners = [Vector(0,0), Vector(width,0), Vector(width,height), Vector(0,height)]
        self.clock = pygame.time.Clock()

        self.scene = Scene(bodies=bodies)
        self.camera = Camera(width=self.width, height=self.height)

        self.debug = False
        self.debug_collisions : list[Collision] = []

    def update(self):
        delta_time = self.clock.tick_busy_loop(self.fps) / 1000

        scroll = 0
        step = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEWHEEL:
                scroll = event.precise_y
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                elif event.key == pygame.K_BACKQUOTE:
                    self.debug = not self.debug
                elif event.key == pygame.K_SPACE:
                    self.scene.paused = not self.scene.paused
                elif event.key == pygame.K_RETURN:
                    self.scene.paused = False
                    step = True

        keys = pygame.key.get_pressed()
        left_click, middle_click, right_click, _, _ = pygame.mouse.get_just_pressed()
        mouse_pos = Vector(*pygame.mouse.get_pos())
        world_pos = self.camera.to_world_space(mouse_pos)
        key = Vector(keys[pygame.K_d] - keys[pygame.K_a], keys[pygame.K_w] - keys[pygame.K_s])
        shift = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]

        self.camera.update(mouse_pos,world_pos,key,shift,scroll,delta_time)

        self.scene.interact(left_click, world_pos)
        self.debug_collisions = self.scene.update(1/self.fps) or self.debug_collisions # fixed delta time

        if step:
            self.scene.paused = True

    def draw_point(self, colour, pos : Vector):
        pygame.draw.aacircle(self.screen, colour, (pos.x, pos.y), 0.1 * self.camera.zoom)

    def draw_line(self, colour, p1 : Vector, p2 : Vector):
        pygame.draw.aaline(self.screen, colour, (p1.x,p1.y), (p2.x,p2.y))

    def draw_AABB(self, colour, p1 : Vector, p2 : Vector):
        x1 = p1.x
        x2 = p2.x
        if x1 > x2:
            x1, x2 = x2, x1

        y1 = p1.y
        y2 = p2.y
        if y1 > y2:
            y1, y2 = y2, y1

        pygame.draw.rect(self.screen, colour, (x1, y1, x2-x1, y2-y1), width=1)

    def draw_plane(self, colour, pos : Vector, norm : Vector):
        points : list[Vector] = []

        for i in range(4):
            p1 = self.corners[i]
            p2 = self.corners[(i+1)%4]

            dp1 = (p1 - pos) * norm
            dp2 = (p2 - pos) * norm

            if (dp1 > 0) ^ (dp2 > 0):
                dpos = p2 - p1
                dpos.normalize()

                t = (pos - p1) * norm / (dpos * norm)

                points.append(p1 + dpos * t)

        if len(points) == 2:
            pygame.draw.aaline(self.screen, colour, (points[0].x, points[0].y), (points[1].x, points[1].y))

    def draw_circle(self, colour, pos : Vector, rad, ang):
        center = (pos.x, pos.y)
        pygame.draw.aacircle(self.screen, colour, center, rad, width=1)
        pygame.draw.aaline(self.screen, colour, center, (pos.x + rad * cos(ang + pi/30), pos.y - rad * sin(ang + pi/30)))
        pygame.draw.aaline(self.screen, colour, center, (pos.x + rad * cos(ang - pi/30), pos.y - rad * sin(ang - pi/30)))

    def draw_polygon(self, colour, points : list[Vector]):
        tuples = [(p.x, p.y) for p in points]
        pygame.gfxdraw.aapolygon(self.screen, tuples, colour)

    def draw(self):
        self.screen.fill(self.bg)

        for body in self.scene.bodies:
            if body.kind == PLANE:
                rel_pos = self.camera.to_screen_space(body.pos)

                self.draw_plane((255,255,255), rel_pos, body.norm)

            elif body.kind == CIRCLE:
                rel_pos = self.camera.to_screen_space(body.pos)
                rel_rad = body.rad * self.camera.zoom

                self.draw_circle((255,255,255), rel_pos, rel_rad, body.ang)

            elif body.kind == POLYGON:
                rel_points = [self.camera.to_screen_space(point) for point in body.transformed_points]
                self.draw_polygon((255,255,255), rel_points)

            if self.debug:
                p1 = self.camera.to_screen_space(Vector(body.AABB.x1,body.AABB.y1))
                p2 = self.camera.to_screen_space(Vector(body.AABB.x2,body.AABB.y2))
                self.draw_AABB((0,255,0),p1,p2)

        if self.debug:
            for collision in self.debug_collisions:
                for p in collision.contacts:
                    rel_pos = self.camera.to_screen_space(p)
                    p1 = self.camera.to_screen_space(p + collision.norm * .5)
                    p2 = self.camera.to_screen_space(p - collision.norm * .5)

                    self.draw_point((255,0,0),rel_pos)
                    self.draw_line((255,0,0), p1, p2)
        
        pygame.display.flip()
        
display = Display()

while 1:
    display.update()
    display.draw()