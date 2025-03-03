from typing import Self

class AABB:
    def __init__(self, x1, y1, x2, y2):
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
        return (self.x1 < other.x1 < other.x2 or self.x1 < other.x2 < other.x2) and (self.y1 < other.y1 < other.y2 or self.y1 < other.y2 < other.y2)
