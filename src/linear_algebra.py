from math import sin, cos

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)
    
    def __mul__(self, other):
        if isinstance(other, Matrix):
            return other * self
        
        if isinstance(other, Vector):
            return self.x * other.x + self.y * other.y
        
        return Vector(self.x * other, self.y * other)
    
    def __rmul__(self, other):
        return self * other
    
    def __truediv__(self, other):
        return Vector(self.x / other, self.y / other)
    
    def __xor__(self, other):
        return self.x * other.y - self.y * other.x
    
    def perpendicular(self):
        return Vector(-self.y, self.x)
    
    def length(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5
    
    def squared_length(self):
        return self.x ** 2 + self.y ** 2
    
    def normalize(self):
        d = self.length()
        if d == 0:
            self.x = 0
            self.y = 1
        else:
            self.x /= d
            self.y /= d

    def normalized(self):
        tmp = Vector(self.x, self.y)
        tmp.normalize()
        return tmp

class Matrix:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def __add__(self, other):
        return Matrix(self.a + other.a, self.b + other.b, self.c + other.c, self.d + other.d)
    
    def __sub__(self, other):
        return Matrix(self.a - other.a, self.b - other.b, self.c - other.c, self.d - other.d)
    
    def __mul__(self, other):
        if isinstance(other, Matrix):
            return Matrix(self.a * other.a + self.c * other.b, 
                          self.b * other.a + self.d * other.b,
                          self.a * other.c + self.c * other.d,
                          self.b * other.c + self.d * other.d)
        
        if isinstance(other, Vector):
            return Vector(other.x * self.a + other.y * self.b, other.x * self.c + other.y * self.d)
        
        return Matrix(self.a * other, self.b * other, self.c * other, self.d * other)
    
    def __truediv__(self, other):
        return Matrix(self.a / other, self.b / other, self.c / other, self.d / other)
    
class RotationMatrix(Matrix):
    def __init__(self, theta, sin_theta=None, cos_theta=None):
        self.theta = theta
        self.sin_theta = sin_theta or sin(self.theta)
        self.cos_theta = cos_theta or cos(self.theta)

        super().__init__(self.cos_theta, -self.sin_theta, self.sin_theta, self.cos_theta)

    def inverse(self):
        return RotationMatrix(-self.theta, -self.sin_theta, self.cos_theta)
