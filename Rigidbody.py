#Rigidbody.py

from Vector import Vector3
from Physics import Equation
import Physics
import PhysicsManager
from ScreenManager import sign

import math

allBodies = []
allHits = []

def hits():
    print("HITS:")
    for hit in allHits:
        print(hit)

#adds a new rigidbody to the system
def addRigidbody(body):
    allBodies.append(body)
    PhysicsManager.addRigidbody(body)

#called by main thread - calculate and carry out appropriate response forces as a result of collisions
def enactCollisions(collisions):
    for c in collisions:
        #obtain average collision point
        point = Vector3(0, 0, 0)
        for p in c.collisionPoints:
            point += Vector3(p)
        point = point / len(c.collisionPoints)
        normalVect1 = -(point - c.body1.position)
        
        normalVect2 = -(point - c.body2.position)
        normalVect1.normalizeSelf()
        normalVect2.normalizeSelf()
        
        rotatedPoint = Vector3(c.body2.position)
        #rotate all into perspective of other body's velocity vector
        horzDistance = math.sqrt(c.body2.velocity[2]**2+c.body2.velocity[0]**2)
        vertDistance = math.sqrt(c.body2.velocity[2]**2+c.body2.velocity[1]**2)
        if horzDistance == 0:
            horzAngle = 0
            if c.body2.velocity[1] < 0:
                vertAngle = -math.pi/2
            if c.body2.velocity[1] > 0:
                vertAngle = math.pi/2
            else:
                vertAngle = 0
        else:
            horzAngle = math.asin(c.body2.velocity[2]/horzDistance)
            vertAngle = math.asin(c.body2.velocity[1]/vertDistance)

        horzDistance = math.sqrt(rotatedPoint[2]**2+rotatedPoint[0]**2)
        currentAngle = math.asin(rotatedPoint[0]/horzDistance)
        if rotatedPoint[2] < 0:
            currentAngle = sign(currentAngle) * math.pi - currentAngle
        rotatedPoint[0] = horzDistance * math.sin(currentAngle-horzAngle)
        rotatedPoint[2] = horzDistance * math.cos(currentAngle-horzAngle)
        vertDistance = math.sqrt(rotatedPoint[2]**2+rotatedPoint[1]**2)
        currentAngle = math.asin(rotatedPoint[1]/vertDistance)
        if rotatedPoint[2] < 0:
            currentAngle = sign(currentAngle) * math.pi - currentAngle
        horzDistance = math.sqrt(rotatedPoint[2]**2+rotatedPoint[0]**2)
        rotatedPoint[1] = horzDistance * math.sin(currentAngle-vertAngle)
        rotatedPoint[2] = horzDistance * math.cos(currentAngle-vertAngle)
        angleFromVelocity = math.asin(math.sqrt(rotatedPoint[0]**2+rotatedPoint[1]**2)/math.sqrt(rotatedPoint[0]**2+rotatedPoint[1]**2+rotatedPoint[2]**2))
        if rotatedPoint[2] < 0:
            angleFromVelocity = sign(angleFromVelocity) * math.pi - angleFromVelocity


        #get rotation vector
        rotationVector = [0,0,0]
        rotationVector[1] = math.sin(angleFromVelocity) * (rotatedPoint[0]/(math.sqrt(rotatedPoint[0]**2+rotatedPoint[1]**2)))
        rotationVector[0] = math.sin(angleFromVelocity) * (rotatedPoint[1]/(math.sqrt(rotatedPoint[0]**2+rotatedPoint[1]**2)))
        rotationVector[2] = math.sin(angleFromVelocity) * (rotatedPoint[1]/(math.sqrt(rotatedPoint[1]**2+rotatedPoint[2]**2)))
        
        dist = math.sqrt(rotatedPoint[0]**2+rotatedPoint[1]**2+rotatedPoint[2]**2)

        #if body is movable, determine normal force vector and apply perfect elastic velocity change
        if c.body1.mass != math.inf and c.body2.mass != math.inf:
            # TODO - potential for rapid/incorrect acceleration on collisions (non-perfect elastic ones?)
                # due to repeat collision calls - solution: move object along normal vect some dist to remove collision before next call
            for i in range(3):
                vel1 = c.body1.velocity[i]
                vel2 = c.body2.velocity[i]
                vf1 = (c.body1.mass*vel1+c.body2.mass*vel2-c.body2.mass*(vel1-vel2))/(c.body1.mass+c.body2.mass)
                vf2 = vel1-vel2+vf1
                c.body1.velocity[i] += (vf1-vel1) * math.cos(angleFromVelocity) * normalVect1[i] * c.body1.elasticity
                c.body1.rotationVelocity[i] = rotationVector[i] *100 * c.body1.elasticity

                c.body2.velocity[i] += (vf2-vel2) * math.cos(angleFromVelocity) * normalVect1[i] * c.body1.elasticity
                c.body2.rotationVelocity[i] = rotationVector[i] *100 * c.body1.elasticity

        elif c.body1.mass == math.inf:
            c.body2.velocity  = -c.body2.velocity * c.body1.elasticity
        elif c.body2.mass == math.inf:
            c.body1.velocity = -c.body1.velocity * c.body1.elasticity


#called by main physics thread to check for rigidbody collisions on all objects
#returns True if any collisions detected, otherwise False
def checkAllCollisions():
    if len(allBodies) < 2:
        return None
    collisions = []
    for i in range(len(allBodies)):
        for j in range(i+1, len(allBodies)):
            dist = math.sqrt(abs(allBodies[i].position[0]-allBodies[j].position[0])**2 + \
                abs(allBodies[i].position[1]-allBodies[j].position[1])**2 + \
                abs(allBodies[i].position[2]-allBodies[j].position[2])**2)
            #bodies are far enough apart that they can't be touching
            if dist > allBodies[i].maxDistanceFromCenter + allBodies[j].maxDistanceFromCenter:
                continue


            #deprecated - unlikely to implement, even the correct version
            #fix equation - calculate normals for faces

            # #bodies are closer enough that they have to be touching
            # if dist < allBodies[i].minDistanceFromCenter + allBodies[j].minDistanceFromCenter:
            #     return True

            #check standard collision
            collisionPoints = checkCollision(allBodies[i], allBodies[j])
            if collisionPoints != None:
                coll = Collision(allBodies[i],allBodies[j],collisionPoints)
                collisions.append(coll)
    if len(collisions) > 0:
        return collisions
    return None



def checkCollision(body1, body2):
    count = 0
    hits = []
    for lineX in body1.uniqueLines:
        for face in body2.faces:
            if face.maxDistance + lineX.maxDistance +1 <= math.sqrt((face.averagePoint[0]-lineX.midpoint[0])**2+\
                                                                (face.averagePoint[1]-lineX.midpoint[1])**2+\
                                                                (face.averagePoint[2]-lineX.midpoint[2])**2):
                continue

            line = Physics.generateEquation(Vector3(lineX.endpoints[0]), Vector3(lineX.endpoints[1]), Physics.PLANE_XY)
            count +=1
            points = [Vector3(face.points[0]),Vector3(face.points[1]),Vector3(face.points[2])]

            offset = [points[0][0], points[0][1], points[0][2]]
            rightAnchor = 0
            for i, p in enumerate(points):
                if p[0] < offset[0]:
                    offset[0] = p[0]
                    offset[1] = p[1]
                    offset[2] = p[2]
            
            newLineEndpoints = [Vector3(line.endpoints[0])-offset, Vector3(line.endpoints[1])-offset]

            #do rotations on each axis to make boundary face flat
            origHorzAngle, origTiltAngle, origVertAngle = [None]*3
            horzTurn, tiltTurn, vertTurn = [None]*3
            for i, point in enumerate(points):
                point = [point[0]-offset[0], point[1]-offset[1], point[2]-offset[2]]
                
                #dont do rotations if current point is offset point
                if abs(point[0]) < 0.00001 and abs(point[1]) < 0.00001 and abs(point[2]) < 0.00001:
                    points[i] = [point[0]+offset[0], point[1]+offset[1], point[2]+offset[2]]
                    continue

                horzDistance = math.sqrt(point[0]**2 + point[2]**2)
                if horzDistance > 0.00001:
                    currentAngle = math.asin(point[0]/horzDistance)
                    if point[2] < 0:
                        currentAngle = math.pi - currentAngle
                    if origHorzAngle == None:
                        rightAnchor = i
                        origHorzAngle = math.asin(point[0]/horzDistance)
                        if point[2] < 0:
                            origHorzAngle = math.pi - origHorzAngle
                        horzTurn = math.pi/2 - origHorzAngle
                    point[0] = horzDistance * math.sin(horzTurn+currentAngle)
                    point[2] = horzDistance * math.cos(horzTurn+currentAngle) 
                tiltDistance = math.sqrt(point[0]**2+point[1]**2)
                if tiltDistance > 0.00001:
                    currentAngle = math.asin(point[1]/tiltDistance)
                    if point[0] < 0:
                        currentAngle = sign(currentAngle)*math.pi - currentAngle
                    if origTiltAngle == None:
                        origTiltAngle = math.asin(point[1]/tiltDistance)
                        if point[0] < 0:
                            origTiltAngle = sign(origTiltAngle)*math.pi - origTiltAngle
                        tiltTurn = -origTiltAngle
                    point[1] = tiltDistance * math.sin(tiltTurn+currentAngle)
                    point[0] = tiltDistance * math.cos(tiltTurn+currentAngle) 
                vertDistance = math.sqrt(point[1]**2+point[2]**2)
                if vertDistance > 0.00001:
                    currentAngle = math.asin(point[1]/vertDistance)
                    if point[2] < 0:
                        currentAngle = sign(currentAngle)*math.pi - currentAngle
                    if origVertAngle == None:
                        origVertAngle = math.asin(point[1]/vertDistance)
                        if point[2] < 0:
                            origVertAngle = sign(origVertAngle)*math.pi - origVertAngle
                        vertTurn = -origVertAngle
                    point[1] = vertDistance * math.sin(vertTurn+currentAngle)
                    point[2] = vertDistance * math.cos(vertTurn+currentAngle) 
                    
                points[i] = [point[0]+offset[0], point[1]+offset[1], point[2]+offset[2]]
            
            #adjust both endpoints of line according to previous rotations
            
            #endpoint1 y-axis
            horzDistance = math.sqrt(newLineEndpoints[0][0]**2+newLineEndpoints[0][2]**2)
            if horzDistance > 0.00001:
                currentAngle = math.asin(newLineEndpoints[0][0]/horzDistance)
                if newLineEndpoints[0][2] < 0:
                    currentAngle = sign(currentAngle) * math.pi - currentAngle
                newLineEndpoints[0][0] = horzDistance * math.sin(horzTurn+currentAngle) 
                newLineEndpoints[0][2] = horzDistance * math.cos(horzTurn+currentAngle) 
            #endpoint2 y-axis
            horzDistance = math.sqrt(newLineEndpoints[1][0]**2+newLineEndpoints[1][2]**2)
            if horzDistance > 0.00001:
                currentAngle = math.asin(newLineEndpoints[1][0]/horzDistance)
                if newLineEndpoints[1][2] < 0:
                    currentAngle = sign(currentAngle) * math.pi - currentAngle
                newLineEndpoints[1][0] = horzDistance * math.sin(horzTurn+currentAngle) 
                newLineEndpoints[1][2] = horzDistance * math.cos(horzTurn+currentAngle) 

            #endpoint1 z-axis
            tiltDistance = math.sqrt(newLineEndpoints[0][0]**2+newLineEndpoints[0][1]**2)
            if tiltDistance > 0.00001:
                currentAngle = math.asin(newLineEndpoints[0][1]/tiltDistance)
                if newLineEndpoints[0][0] < 0:
                    currentAngle = sign(currentAngle) * math.pi - currentAngle
                newLineEndpoints[0][1] = tiltDistance * math.sin(tiltTurn+currentAngle) 
                newLineEndpoints[0][0] = tiltDistance * math.cos(tiltTurn+currentAngle) 
            #endpoint2 z-axis
            tiltDistance = math.sqrt(newLineEndpoints[1][0]**2+newLineEndpoints[1][1]**2)
            if tiltDistance > 0.00001:
                currentAngle = math.asin(newLineEndpoints[1][1]/tiltDistance)
                if newLineEndpoints[1][0] < 0:
                    currentAngle = sign(currentAngle) * math.pi - currentAngle
                newLineEndpoints[1][1] = tiltDistance * math.sin(tiltTurn+currentAngle) 
                newLineEndpoints[1][0] = tiltDistance * math.cos(tiltTurn+currentAngle) 

            #endpoint1 x-axis
            vertDistance = math.sqrt(newLineEndpoints[0][1]**2+newLineEndpoints[0][2]**2)
            if vertDistance > 0.00001:
                currentAngle = math.asin(newLineEndpoints[0][1]/vertDistance)
                if newLineEndpoints[0][2] < 0:
                    currentAngle = sign(currentAngle) * math.pi - currentAngle
                newLineEndpoints[0][1] = vertDistance * math.sin(vertTurn+currentAngle) 
                newLineEndpoints[0][2] = vertDistance * math.cos(vertTurn+currentAngle) 
            #endpoint2 x-axis
            vertDistance = math.sqrt(newLineEndpoints[1][1]**2+newLineEndpoints[1][2]**2)
            if vertDistance > 0.00001:
                currentAngle = math.asin(newLineEndpoints[1][1]/vertDistance)
                if newLineEndpoints[1][2] < 0:
                    currentAngle = sign(currentAngle) * math.pi - currentAngle
                newLineEndpoints[1][1] = vertDistance * math.sin(vertTurn+currentAngle) 
                newLineEndpoints[1][2] = vertDistance * math.cos(vertTurn+currentAngle) 
                
            newLineEndpoints[0]+=offset
            newLineEndpoints[1]+=offset
            line = Physics.generateEquation(newLineEndpoints[0],newLineEndpoints[1],Physics.PLANE_XY)
            
            #create horizontal line segment from outermost (x-axis) points
            pointsOrdered = sortPointsByAxis(points, Physics.AXIS_X)
            boundary = Physics.generateEquation(pointsOrdered[0], pointsOrdered[-1], Physics.PLANE_XY)

            #check if intersection between flat plane boundary face and line

            #line is just point
            if line.coefficient == None:
                x = line.endpoints[0][0]
                y = line.endpoints[0][1]
            #line has vertical slope
            elif line.coefficient == math.inf:
                x = line.endpoints[0][0]
                y = boundary.endpoints[0][1]
            #line is horizontal (parallel)
            elif abs(line.coefficient - boundary.coefficient) <= 0.00001:
                x = line.endpoints[0][0]
                y = boundary.endpoints[0][1]
            #line is regular    
            else:
                x = (boundary.intercept - line.intercept) / \
                 (line.coefficient - boundary.coefficient)
                y = boundary.coefficient * x + boundary.intercept
            #ensure intersection is within bounds                
            if x <= pointsOrdered[-1][0] and x >= pointsOrdered[0][0] and \
                    x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                    x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                    y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                    y >= min(line.endpoints[0][1], line.endpoints[1][1]):

                #change the two boundaries with intersection into XZ plane
                if x < pointsOrdered[1][0]:
                    seg1 = Physics.generateEquation(pointsOrdered[0],pointsOrdered[1], Physics.PLANE_XZ)
                else:
                    seg1 = Physics.generateEquation(pointsOrdered[1],pointsOrdered[-1], Physics.PLANE_XZ)
                seg2 = Physics.generateEquation(pointsOrdered[0],pointsOrdered[-1], Physics.PLANE_XZ)
                z1 = seg1.evaluate(x=x)
                z2 = seg2.evaluate(x=x)
                if abs(z1) == math.inf:
                    z1 = seg1.endpoints[0][2]
                    z2 = seg1.endpoints[1][2]
                elif abs(z2) == math.inf:
                    z1 = seg2.endpoints[0][2]
                    z2 = seg2.endpoints[1][2]
                    
                #create boundary in XZ plane connecting two points from cross-section of intersection
                boundaryXZ = Physics.generateEquation((x,y,z1), \
                                                    (x,y,z2), Physics.PLANE_XZ)

                #change line into XZ plane
                line = Physics.generateEquation(line.endpoints[0], line.endpoints[1], Physics.PLANE_XZ)

                #check for intersection between new line and boundary in XZ plane
                z = line.evaluate(x=x)
                
                #ensure intersection is within bounds
                if z <= max(boundaryXZ.endpoints[0][2], boundaryXZ.endpoints[1][2]) and \
                        z >= min(boundaryXZ.endpoints[0][2], boundaryXZ.endpoints[1][2]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        z <= max(line.endpoints[0][2], line.endpoints[1][2]) and \
                        z >= min(line.endpoints[0][2], line.endpoints[1][2]):
                    hits.append((x, y, z))
    i=1
    while (i < len(hits)):
        for p in hits[0:i]:
            if abs(hits[i][0] - p[0]) <= 0.00001 and \
                abs(hits[i][1] - p[1]) <= 0.00001 and \
                abs(hits[i][2] - p[2]) <= 0.00001:
                del(hits[i])
                i-=1
                break
        i+=1
    if len(hits) != 0:
        return hits
    return None


#generate equations defining the points bounding a face
#(FVPs) faceVerticesPos - ex:
#   [(0, 0, 0), (1, 1, 1), (2, 2, 2)]   <- face 1
def generateFaceBoundaryEquations(FVPs, averagePoint, plane, parent=None):
    lis = []
    
    for i in range(len(FVPs)):
        if i+1 == len(FVPs):
            eq = Physics.generateEquation(Vector3(FVPs[i]), Vector3(FVPs[0]), plane, parent)
        else:
            eq = Physics.generateEquation(Vector3(FVPs[i]), Vector3(FVPs[i+1]), plane, parent)

        if eq.coefficient == None:
            sign = Equation.EQUAL
        elif plane == Physics.PLANE_XY:
            if eq.coefficient == math.inf:
                if (eq.endpoints[0][0] > averagePoint[0]):
                    sign = Equation.LESS_THAN_EQUAL_TO
                elif (eq.endpoints[0][0] < averagePoint[0]):
                    sign = Equation.GREATER_THAN_EQUAL_TO
                else:
                    sign = Equation.EQUAL
            elif (eq.coefficient * averagePoint[0] + eq.intercept) > averagePoint[1]:
                sign = Equation.LESS_THAN_EQUAL_TO
            elif (eq.coefficient * averagePoint[0] + eq.intercept) < averagePoint[1]:
                sign = Equation.GREATER_THAN_EQUAL_TO
            else:
                sign = Equation.EQUAL
        elif plane == Physics.PLANE_XZ:
            if eq.coefficient == math.inf:
                if (eq.endpoints[0][0] > averagePoint[0]):
                    sign = Equation.LESS_THAN_EQUAL_TO
                elif (eq.endpoints[0][0] < averagePoint[0]):
                    sign = Equation.GREATER_THAN_EQUAL_TO
                else:
                    sign = Equation.EQUAL
            elif (eq.coefficient * averagePoint[0] + eq.intercept) > averagePoint[2]:
                sign = Equation.LESS_THAN_EQUAL_TO
            elif (eq.coefficient * averagePoint[0] + eq.intercept) < averagePoint[2]:
                sign = Equation.GREATER_THAN_EQUAL_TO
            else:
                sign = Equation.EQUAL
        eq.comparator = sign
        lis.append(eq)
    
    return lis

#returns list with items ordered ascending
def insertionSort(arr):
    key, j = 0, 0
    for i in range(1, len(arr)):
        key = arr[i]
        j = i-1
        while j >= 0 and arr[j] > key:
            arr[j+1] = arr[j]
            j -= 1
        arr[j+1] = key
    return arr

#returns list of points with points ordered by the specified axis value ascending
def sortPointsByAxis(arr, axis):
    if axis == Physics.AXIS_X:
        for i in range(1, len(arr)):
            key = arr[i]
            j = i-1
            while j >= 0 and arr[j][0] > key[0]:
                arr[j+1] = arr[j]
                j -= 1
            arr[j+1] = key
    elif axis == Physics.AXIS_Y:
        for i in range(1, len(arr)):
            key = arr[i]
            j = i-1
            while j >= 0 and arr[j][1] > key[1]:
                arr[j+1] = arr[j]
                j -= 1
            arr[j+1] = key
    elif axis == Physics.AXIS_Z:
        for i in range(1, len(arr)):
            key = arr[i]
            j = i-1
            while j >= 0 and arr[j][2] > key[2]:
                arr[j+1] = arr[j]
                j -= 1
            arr[j+1] = key
    return arr

class Rigidbody:
    def __init__(self, parent, position=None,
                 velocity=Vector3(0, 0, 0),
                 acceleration=Vector3(0, 0, 0),
                 mass=1,
                 airResistance=0.01,
                 gravity=1,
                 elasticity=0.8):
        self.parent = parent
        if position is None:
            self.position = Vector3(parent.position)
        else:
            self.position = Vector3(position)
        self.velocity = velocity
        self.acceleration = acceleration
        self.rotationVelocity = Vector3(0, 0, 0)
        self.facesVerticesPos = parent.getFaceVerticesRelPos()
        self.uniqueLines = []
        self.faces = []
        self.boundaryEquations = []
        for face in parent.triangulate(parent.getFaceVerticesPos()):
            averagePoint = [0, 0 ,0]
            for point in face:
                averagePoint[0] += point[0]
                averagePoint[1] += point[1]
                averagePoint[2] += point[2]
            averagePoint[0] /= len(face)
            averagePoint[1] /= len(face)
            averagePoint[2] /= len(face)
            newBoundary = generateFaceBoundaryEquations(face, averagePoint, Physics.PLANE_XY)
            self.faces.append(Face(face, averagePoint=averagePoint))
            self.boundaryEquations.append(newBoundary)
            for line in newBoundary:
                if line not in self.uniqueLines:
                    self.uniqueLines.append(line)
        vectsFromCenter = [Vector3(point) for face in self.facesVerticesPos for point in face]
        distsFromCenter = [math.sqrt(vect[0]**2+vect[1]**2+vect[2]**2) for vect in vectsFromCenter]
        distsFromCenter = insertionSort(distsFromCenter)
        self.maxDistanceFromCenter = max(distsFromCenter)
        # for face in self.boundaryEquations:
        #     for line in face:

        # self.minDistanceFromCenter = min(distsFromCenter)
        self.gravity = gravity
        self.airResistance = airResistance
        self.mass = mass
        self.elasticity = elasticity

    def resetBoundaries(self):
        self.boundaryEquations = []
        for face in self.parent.getFaceVerticesPos():
            averagePoint = [0, 0 ,0]
            for point in face:
                averagePoint[0] += point[0]
                averagePoint[1] += point[1]
                averagePoint[2] += point[2]
            averagePoint[0] /= len(face)
            averagePoint[1] /= len(face)
            averagePoint[2] /= len(face)
            self.boundaryEquations.append(generateFaceBoundaryEquations(
                            face, averagePoint, Physics.PLANE_XY))


    def distanceFrom(body):
        distVect = self.position - body.position
        return math.sqrt(math.sqrt(distVect[0]**2 + distVect[1]**2) + distVect[2]**2)

class Face:
    def __init__(self, points, averagePoint=None):
        self.points = points
        if averagePoint != None:
            self.averagePoint = averagePoint
        else:
            self.averagePoint = [0, 0 ,0]
            for point in points:
                averagePoint[0] += point[0]
                averagePoint[1] += point[1]
                averagePoint[2] += point[2]
            self.averagePoint[0] /= 3
            self.averagePoint[1] /= 3
            self.averagePoint[2] /= 3
        self.maxDistance =  max(math.sqrt((points[0][0]-averagePoint[0])**2+(points[0][1]-averagePoint[1])**2+(points[0][2]-averagePoint[2])**2),\
                                math.sqrt((points[1][0]-averagePoint[0])**2+(points[1][1]-averagePoint[1])**2+(points[1][2]-averagePoint[2])**2),\
                                math.sqrt((points[2][0]-averagePoint[0])**2+(points[2][1]-averagePoint[1])**2+(points[2][2]-averagePoint[2])**2))

    def __eq__(self, other):
        for point in other.points:
            if point not in points:
                return False
        return True

    def __getitem__(self, key):
        return self.points[key]

    def __str__(self):
        return str(self.points)


class Collision:
    def __init__(self, body1, body2, collisionPoints):
        self.body1 = body1
        self.body2 = body2
        self.collisionPoints = collisionPoints




        

    
        
