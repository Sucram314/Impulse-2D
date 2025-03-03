import timeit

setup1 = """
from math import sin, cos
"""

code1 = """
a = sin(832843)
b = cos(832843)
"""

setup2 = """
from math import e
"""

code2 = """
x = e ** complex(0,832843)
a = x.real
b = x.imag
"""

print(timeit.timeit(code1, setup1))
print(timeit.timeit(code2, setup2))