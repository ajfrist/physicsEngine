#Vector.py

import math

class Vector3:
    def __init__(self, *args):
        self.current = -1
        if isinstance(args[0], tuple) or isinstance(args[0], list) or isinstance(args[0], Vector3):
            self.x = args[0][0]
            self.y = args[0][1]
            self.z = args[0][2]
        else:
            self.x = args[0]
            self.y = args[1]
            self.z = args[2]
        self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2)

        
    def __str__(self):
        return f"Vector3({self.x}, {self.y}, {self.z})"

    def __repr__(self):
        return f"Vector3({self.x}, {self.y}, {self.z})"

    def __getitem__(self, key):
        if key==0:
            return self.x
        elif key==1:
            return self.y
        elif key==2:
            return self.z
        else:
            return None

    def __setitem__(self, key, value):
        if key==0:
            self.x = value
            self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        elif key==1:
            self.y = value
            self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        elif key==2:
            self.z = value
            self.magnitude = math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalizeSelf(self):
        self.x /= self.magnitude
        self.y /= self.magnitude
        self.z /= self.magnitude
        self.magnitude = 1

    def setMag(self, mag):
        self.x *= mag
        self.y *= mag
        self.z *= mag
        self.magnitude = mag

    def __add__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        elif (isinstance(other, tuple) or isinstance(other, list)) and len(other) == 3:
            return Vector3(self.x + other[0], self.y + other[1], self.z + other[2])
        elif isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x + other, self.y + other, self.z + other)
        else:
            raise TypeError("Unsupported operand type. Can only add Vector3 objects.")

    def __radd__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        elif (isinstance(other, tuple) or isinstance(other, list)) and len(other) == 3:
            return Vector3(self.x + other[0], self.y + other[1], self.z + other[2])
        elif isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x + other, self.y + other, self.z + other)
        else:
            raise TypeError("Unsupported operand type. Can only add Vector3 objects.")

    def __iadd__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        elif (isinstance(other, tuple) or isinstance(other, list)) and len(other) == 3:
            return Vector3(self.x + other[0], self.y + other[1], self.z + other[2])
        elif isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x + other, self.y + other, self.z + other)
        else:
            raise TypeError("Unsupported operand type. Can only add Vector3 objects.")

    def __sub__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            raise TypeError("Unsupported operand type. Can only subtract Vector3 objects.")

    def __mul__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        elif isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x * other, self.y * other, self.z * other)
        else:
            raise TypeError("Unsupported operand type. Can only multiply Vector3 objects.")

    def __rmul__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        elif isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x * other, self.y * other, self.z * other)
        else:
            raise TypeError("Unsupported operand type. Can only multiply Vector3 objects.")

    def __truediv__(self, other):
        if isinstance(other, Vector3):
            if other.x != 0 and other.y != 0:
                return Vector3(self.x / other.x, self.y / other.y, self.z / other.z)
            else:
                raise ValueError("Division by zero not allowed.")
        elif isinstance(other, float) or isinstance(other, int):
            return Vector3(self.x / other, self.y / other, self.z / other)
        else:
            raise TypeError("Unsupported operand type. Can only divide Vector3 objects.")

    def __neg__(self):
        return Vector3(-self.x, -self.y, -self.z)

    def __iter__(self):
        self.current = -1
        return self

    def __next__(self):
        if self.current >= 2:
            raise StopIteration
        self.current += 1
        if self.current == 0:
            return self.x
        elif self.current == 1:
            return self.y
        elif self.current == 2:
            return self.z

    def getMag(self):
        return self.magnitude


class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.magnitude = math.sqrt(self.x**2 + self.y**2)

    def __str__(self):
        return f"Vector2(x={self.x}, y={self.y})"

    def __getitem__(self, key):
        if key==0:
            return self.x
        elif key==1:
            return self.y
        else:
            return None

    def __setitem__(self, key, value):
        if key==0:
            self.x = value
            self.magnitude = math.sqrt(self.x**2 + self.y**2)
        elif key==1:
            self.y = value
            self.magnitude = math.sqrt(self.x**2 + self.y**2)

    def normalizeSelf(self):
        self.x /= magnitude
        self.y /= magnitude
        self.magnitude = 1

    def setMag(self, mag):
        self.x *= mag
        self.y *= mag
        self.magnitude = mag

    def __add__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x, self.y + other.y)
        else:
            raise TypeError("Unsupported operand type. Can only add Vector2 objects.")

    def __sub__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x - other.x, self.y - other.y)
        else:
            raise TypeError("Unsupported operand type. Can only subtract Vector2 objects.")

    def __mul__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x * other.x, self.y * other.y)
        else:
            raise TypeError("Unsupported operand type. Can only multiply Vector2 objects.")

    def __div__(self, other):
        if isinstance(other, Vector2):
            if other.x != 0 and other.y != 0:
                return Vector2(self.x / other.x, self.y / other.y)
            else:
                raise ValueError("Division by zero not allowed.")
        else:
            raise TypeError("Unsupported operand type. Can only divide Vector2 objects.")


    def getMag(self):
        return self.magnitude

        
