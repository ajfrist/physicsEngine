#Physics.py

import math

from Vector import Vector3
from Constants import PHYSICS_FPS
from PhysicsManager import ALL_RIGIDBODIES

AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 2
PLANE_XY = 10
PLANE_XZ = 11
PLANE_YZ = 12

class Force:
    def __init__(self, vector, duration, target):
        self.vector = vector          #vector direction
        self.duration = duration  #duration of force in seconds
        if target != Physics.ALL_RIGIDBODIES:
            self.target = target      #rigidbody to apply force to

    #updates the target with the applied force
    def applyForce(self):
        target.updatePosition(self.vector * (1 / PHYSICS_FPS))

def generateEquation(point1, point2, plane, parent=None):
    if plane == PLANE_XY:
        if abs(point1[0]-point2[0]) > 0.00001:
            slope = (point1[1]-point2[1])/(point1[0]-point2[0])
        elif abs(point1[1]-point2[1]) <= 0.00001:
            slope = None
        else:
            slope = math.inf
        if slope == None or slope == math.inf:
            intercept = None
        else:
            intercept = slope * -point1[0] + point1[1]
    if plane == PLANE_XZ:
        if abs(point1[0]-point2[0]) > 0.00001:
            slope = (point1[2]-point2[2])/(point1[0]-point2[0])
        elif abs(point1[2]-point2[2]) <= 0.00001:
            slope = None
        else:
            slope = math.inf
        if slope == None or slope == math.inf:
            intercept = None
        else:
            intercept = slope * -point1[0] + point1[2]
    if plane == PLANE_YZ:
        if abs(point1[2]-point2[2]) > 0.00001:
            slope = (point1[1]-point2[1])/(point1[2]-point2[2])
        elif abs(point1[1]-point2[1]) <= 0.00001:
            slope = None
        else:
            slope = math.inf
        if slope == None or slope == math.inf:
            intercept = None
        else:
            intercept = slope * -point1[2] + point1[1]
    return Equation(coefficient=slope, intercept=intercept, endpoints=[point1,point2], plane=plane, parent=parent)

class Equation:
    LESS_THAN = 0
    LESS_THAN_EQUAL_TO = 1
    EQUAL = 2
    GREATER_THAN_EQUAL_TO = 3
    GREATER_THAN = 4
    
    def __init__(self, comparator=EQUAL, coefficient=1, intercept=0, endpoints=[(0,0,0),(1,1,1)], plane=PLANE_XY, parent=None):
        self.comparator = comparator
        self.coefficient = coefficient
        self.intercept = intercept
        self.endpoints = endpoints
        self.plane = plane
        self.parent = parent

    def evaluate(self, x=None, y=None):
        if x != None and y != None:
            result = self.coefficient * x + self.intercept
            if self.comparator == Equation.LESS_THAN:
                return y < result
            elif self.comparator == Equation.LESS_THAN_EQUAL_TO:
                return y <= result
            elif self.comparator == Equation.GREATER_THAN:
                return y > result
            elif self.comparator == Equation.GREATER_THAN_EQUAL_TO:
                return y >= result
            else:
                raise ValueError("Must only specify one argument when comparator is EQUAL.")
        # if self.comparator != Equation.EQUAL:
        #     raise ValueError("Must specify two arguments when comparator is not EQUAL.")
        if x != None:
            if self.coefficient == None:
                if self.plane == PLANE_XY:
                    return self.endpoints[0][1]
                elif self.plane == PLANE_XZ:
                    return self.endpoints[0][2]
            if self.coefficient == math.inf:
                return math.inf * x
            return self.coefficient * x + self.intercept
        if y != None:
            if self.coefficient == None:
                return self.endpoints[0][0]
            if self.coefficient == math.inf:
                return None
            return (y - self.intercept) / self.coefficient
        raise ValueError("Must specify an argument to evaluate.")

    def endpoints(self):
        return [self.endPoints[0]+self.position[0],self.endPoints[1]+self.position[1],self.endPoints[2]+self.position[2]]
            
    

