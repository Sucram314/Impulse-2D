from typing import Self
from .linear_algebra import Vector

INF = float("inf")

class AABB:
    def __init__(self, x1=-INF, y1=-INF, x2=INF, y2=INF):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def update(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def collide(self, other : Self):
        return self.x1 < other.x2 and self.x2 > other.x1 and self.y1 < other.y2 and self.y2 > other.y1