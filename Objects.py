#Objects.py

import pywavefront
import math
from Vector import Vector3
import ScreenManager
from ScreenManager import sign, toRad, toDeg
import Physics
from Constants import *

class Object:
    def __init__(self, position=(0, 0, 0), rotation=[0,0,0]):
        self.position = Vector3(position)
        self.rotation = rotation
        self.rigidbody = None
        self.parent = None
        self.children = []
        self.rotationLock = False

    def setPosition(self, pos):
        self.position = pos

    #move the object in world space
    def move(self, vector3):
        self.position[0] += vector3[0]
        self.position[1] += vector3[1]
        self.position[2] += vector3[2]
        if self.rigidbody != None:
            self.rigidbody.position[0] += vector3[0]
            self.rigidbody.position[1] += vector3[1]
            self.rigidbody.position[2] += vector3[2]

    #move the object in relative space on only the horizontal (XZ) plane
    def moveRelHorz(self, vector3):
        magnitude = math.sqrt(vector3[0]**2+vector3[2]**2)
        if vector3[2] != 0:
            vAngle = math.atan(vector3[0]/vector3[2])
            if vector3[2] < 0:
                vAngle += math.pi
        elif vector3[0] != 0:
            vAngle = vector3[0]/abs(vector3[0]) * math.pi/2
        else:
            vAngle = 0
        angle = self.rotation[0]/180*math.pi + vAngle

        if self.rotationLock:
            self.position[0] += magnitude * math.sin(angle)
            self.position[1] += vector3[1]
            self.position[2] += magnitude * math.cos(angle)
            if self.rigidbody != None:
                self.rigidbody.position[0] += magnitude * math.sin(angle)
                self.rigidbody.position[1] += vector3[1]
                self.rigidbody.position[2] += magnitude * math.cos(angle)

    def setRotation(self, angle):
        self.rotation[0] = angle[0]
        self.rotation[1] = angle[1]
        while self.rotation[0] >= 360:
            self.rotation[0] -= 360
        while self.rotation[0] < 0:
            self.rotation[0] += 360
        if angle[1] > 90:
            angle[1] = 90
        elif angle[1] < -90:
            angle[1] = -90

    def rotate(self, angle):
        if self.rotationLock:
            self.rotation[0] += angle[0]
            while self.rotation[0] > 360:
                self.rotation[0] -= 360
            while self.rotation[0] <= 0:
                self.rotation[0] += 360
            
            self.rotation[1] += angle[1]
            if self.rotation[1] > 90:
                self.rotation[1] = 90
            elif self.rotation[1] < -90:
                self.rotation[1] = -90
        else:
            self.rotation[0] += angle[0]
            while self.rotation[0] > 360:
                self.rotation[0] -= 360
            while self.rotation[0] <= 0:
                self.rotation[0] += 360
            self.rotation[1] += angle[1]
            while self.rotation[1] > 360:
                self.rotation[1] -= 360
            while self.rotation[1] <= 0:
                self.rotation[1] += 360
            self.rotation[2] += angle[2]
            while self.rotation[2] > 360:
                self.rotation[2] -= 360
            while self.rotation[2] <= 0:
                self.rotation[2] += 360

    def setRigidbody(self, body):
        self.rigidbody = body

    def getRigidbody(self):
        return self.rigidbody

    def distanceFrom(self, obj):
        distVect = self.position - obj.position
        return math.sqrt(math.sqrt(distVect[0]**2 + distVect[1]**2) + distVect[2]**2)

    def addChild(self, child):
        self.children.append(child)

    
class MeshObject(Object):
    def __init__(self, position, fileName=None, borderColor=BLACK, fillColor=WHITE):
        super().__init__(position)
        self.borderColor = borderColor
        self.fillColor = fillColor
        if (fileName != None):
            self.obj = pywavefront.Wavefront(fileName, collect_faces=True)
            #list of each vertex's position in world space
            self.vertices = obj.vertices
            self.vertRotations = []
            for vert in self.vertices:
                angleX = 180/math.pi*math.asin(vert[0]/math.sqrt(vert[0]**2+vert[2]**2))
                angleY = 180/math.pi*math.asin(vert[1]/math.sqrt(vert[0]**2+vert[1]**2+vert[2]**2))
                angleZ = 0
                if vert[2] >= 0:
                    self.vertRotations.append([angleX,angleY,angleZ])
                else:
                    self.vertRotations.append([(angleX/abs(angleX))*180-angleX,angleY,angleZ])
            self.meshes = [mesh for name, mesh in obj.meshes.items()]
            #list of each face's coresponding vertex indeces
            self.faceVertexIndices = [face for face in mesh.faces for mesh in meshes]

    def move(self, vector3):
        super().move(vector3)
        if self.rigidbody != None:
            for face in self.rigidbody.boundaryEquations:
                for line in face:
                    line.endpoints[0] += vector3
                    line.endpoints[1] += vector3


# local rotate:
# get point in local space
# per axis, apply local rotation
# get point location from rotation
# save new location
# save new rotation
    def rotate(self, angle):
        if self.rotationLock:
            self.rotation[0] += angle[0]
            while self.rotation[0] > 360:
                self.rotation[0] -= 360
            while self.rotation[0] <= 0:
                self.rotation[0] += 360
            
            self.rotation[1] += angle[1]
            if self.rotation[1] > 90:
                self.rotation[1] = 90
            elif self.rotation[1] < -90:
                self.rotation[1] = -90
        else:
            self.rotation[0] += angle[0]
            while self.rotation[0] > 360:
                self.rotation[0] -= 360
            while self.rotation[0] <= 0:
                self.rotation[0] += 360


            self.rotation[1] += angle[1]
            while self.rotation[1] > 360:
                self.rotation[1] -= 360
            while self.rotation[1] <= 0:
                self.rotation[1] += 360


            self.rotation[2] += angle[2]
            while self.rotation[2] > 360:
                self.rotation[2] -= 360
            while self.rotation[2] <= 0:
                self.rotation[2] += 360

# world rotate:
# get point in local space
# per axis, apply world rotation
# get local rotation from new location
# save new location
# save new rotation
    def rotateWorld(self, angle):
        if not self.rotationLock:
            for i, point in enumerate(self.vertices):
                startPos = [point[0],point[1],point[2]]

                newAngleX = 180/math.pi*math.asin(startPos[1]/math.sqrt(startPos[1]**2+startPos[2]**2))
                newAngleY = 180/math.pi*math.asin(startPos[0]/math.sqrt(startPos[0]**2+startPos[2]**2))
                newAngleZ = 180/math.pi*math.asin(startPos[1]/math.sqrt(startPos[0]**2+startPos[1]**2))
                if startPos[2] < 0:
                    newAngleX = sign(newAngleX)*180 - newAngleX
                    newAngleY = sign(newAngleY)*180 - newAngleY
                if startPos[0] < 0:
                    newAngleZ = sign(newAngleZ)*180 - newAngleZ
                rotation = [newAngleX,newAngleY,newAngleZ]

                #adjust x rotation
                angleX = rotation[0]+angle[0]
                mag = math.sqrt(startPos[1]**2+startPos[2]**2)
                startPos[1] += (math.sin(toRad(angleX)) - math.sin(toRad(rotation[0]))) * mag
                startPos[2] += (math.cos(toRad(angleX)) - math.cos(toRad(rotation[0]))) * mag

                newAngleX = 180/math.pi*math.asin(startPos[1]/math.sqrt(startPos[1]**2+startPos[2]**2))
                newAngleY = 180/math.pi*math.asin(startPos[0]/math.sqrt(startPos[0]**2+startPos[2]**2))
                newAngleZ = 180/math.pi*math.asin(startPos[1]/math.sqrt(startPos[0]**2+startPos[1]**2))
                if startPos[2] < 0:
                    newAngleX = sign(newAngleX)*180 - newAngleX
                    newAngleY = sign(newAngleY)*180 - newAngleY
                if startPos[0] < 0:
                    newAngleZ = sign(newAngleZ)*180 - newAngleZ
                rotation = [newAngleX,newAngleY,newAngleZ]

                #adjust y rotation
                angleY = rotation[1]+angle[1]
                mag = math.sqrt(startPos[0]**2+startPos[2]**2)
                startPos[0] += (math.sin(toRad(angleY)) - math.sin(toRad(rotation[1]))) * mag
                startPos[2] += (math.cos(toRad(angleY)) - math.cos(toRad(rotation[1]))) * mag

                newAngleX = 180/math.pi*math.asin(startPos[1]/math.sqrt(startPos[1]**2+startPos[2]**2))
                newAngleY = 180/math.pi*math.asin(startPos[0]/math.sqrt(startPos[0]**2+startPos[2]**2))
                newAngleZ = 180/math.pi*math.asin(startPos[1]/math.sqrt(startPos[0]**2+startPos[1]**2))
                if startPos[2] < 0:
                    newAngleX = sign(newAngleX)*180 - newAngleX
                    newAngleY = sign(newAngleY)*180 - newAngleY
                if startPos[0] < 0:
                    newAngleZ = sign(newAngleZ)*180 - newAngleZ
                rotation = [newAngleX,newAngleY,newAngleZ]

                #adjust z rotation
                angleZ = rotation[2]+angle[2]
                mag = math.sqrt(startPos[0]**2+startPos[1]**2)
                startPos[1] += (math.sin(toRad(angleZ)) - math.sin(toRad(rotation[2]))) * mag
                startPos[0] += (math.cos(toRad(angleZ)) - math.cos(toRad(rotation[2]))) * mag

                self.vertices[i]=(startPos[0],startPos[1],startPos[2])


    #takes index of face or face and returns list of vertices used in face in world space
    #basic faces -- might break with faces that have uv and texture data
    #
    #returns list of each face's corresponding vertex positions in world space
    #  if faceIndex is not None, returns only the points for the face requested
    #example:
    #   [[(0, 0, 0), (1, 1, 1), (2, 2, 2)],  <- face 1
    #    [(0, 0, 0), (2, 2, 2), (3, 3, 3)],  <- face 2
    #    [(1, 1, 1), (2, 2, 2), (3, 3, 3)],  <- face 3
    #    [(0, 0, 0), (1, 1, 1), (3, 3, 3)]]  <- face 4
    #
    #faceGiven - list (tuple) of indeces of verteces used
    #   ex:   (0, 2, 3, 1)  <- face 1
    #   returns:
    #       [(0, 0, 0), (1, 1, 1), (2, 2, 2)]  <- face 1
    def getFaceVerticesPos(self, faceIndex=None, faceGiven=None):
        if faceGiven != None:
            return [[pos + self.position[j] for j,pos in enumerate(self.vertices[vertexIndex])] for vertexIndex in faceGiven]
        return [[[pos + self.position[j] for j,pos in enumerate(self.vertices[pIndex])] for pIndex in face] for face in self.faceVertexIndices]
        
    def getFaceVerticesRelPos(self):
        return [[[pos for j,pos in enumerate(self.vertices[pIndex])] for pIndex in face] for face in self.faceVertexIndices]

    #returns identical list of faces, except with quads replaced with two triangles
    #does not support faces larger than quads -- will return original list of faces
    def triangulate(self, listFaceVertexIndices):
        trangulatedFaceVertexIndices = []
        for face in listFaceVertexIndices:
            if size(face) > 4:
                return listFaceVertexIndices
            elif size(face) > 3:
                trangulatedFaceVertexIndices.append((face[0], face[1], face[2]))
                trangulatedFaceVertexIndices.append((face[3], face[0], face[2]))
            else:
                trangulatedFaceVertexIndices.append(face)
        return trangulatedFaceVertexIndices


    #returns list of each face's corresponding vertex indices used
    #example:
    #
    #   [(0, 1, 2),  <- face 1
    #    (0, 2, 3),  <- face 2
    #    (1, 2, 3),  <- face 3
    #    (0, 1, 3)]  <- face 4
    def getFaceVertexIndices(self):
        return self.faceVertexIndices

    #returns list of each face's corresponding vertex positions in world space
    #  if faceIndex is not None, returns only the points for the face requested
    #example:
    #
    #   [[(0, 0, 0), (1, 1, 1), (2, 2, 2)],  <- face 1
    #    [(0, 0, 0), (2, 2, 2), (3, 3, 3)],  <- face 2
    #    [(1, 1, 1), (2, 2, 2), (3, 3, 3)],  <- face 3
    #    [(0, 0, 0), (1, 1, 1), (3, 3, 3)]]  <- face 4
##    def getFacePoints(self):
##        return [for face in self.]


    def getFaceCount(self):
        return len(self.faceVertexIndices)
    

class Cube(MeshObject):
    def __init__(self, position, sideLength, borderColor=BLACK, fillColor=BLACK):
        super().__init__(position, borderColor=borderColor, fillColor=fillColor)
        self.sideLength = sideLength

        factor = 1 / (sideLength/4)
        self.vertices = []
        for x in range(int(factor*-sideLength/2), int(factor*sideLength), int(factor*sideLength)):
            for y in range(int(factor*-sideLength/2), int(factor*sideLength), int(factor*sideLength)):
                for z in range(int(factor*-sideLength/2), int(factor*sideLength), int(factor*sideLength)):
                    self.vertices.append((x/factor, y/factor, z/factor))

        self.vertices2 = [vert for vert in self.vertices]

        self.vertRotations = []
        for vert in self.vertices:
            angleX = 180/math.pi*math.asin(vert[0]/math.sqrt(vert[0]**2+vert[2]**2))
            angleY = 180/math.pi*math.asin(vert[1]/math.sqrt(vert[0]**2+vert[1]**2+vert[2]**2))
            angleZ = 0
            if vert[2] >= 0:
                self.vertRotations.append([angleX,angleY,angleZ])
            else:
                self.vertRotations.append([(angleX/abs(angleX))*180-angleX,angleY,angleZ])

        #                           left (-x)    bottom (-y)   right (+x)     top (+y)     back (+z)     front (-z)
        self.faceVertexIndices = [(0, 1, 3, 2), (0, 1, 5, 4), (4, 5, 7, 6), (3, 7, 6, 2), (3, 1, 5, 7), (0, 2, 6, 4)]

class Plane(MeshObject):
    def __init__(self, position, length, width, plane=Physics.PLANE_XZ, borderColor=BLACK, fillColor=BLACK):
        super().__init__(position, borderColor=borderColor, fillColor=fillColor)
        self.length = length
        self.width = width
        self.plane = plane

        if plane == Physics.PLANE_XY:
            self.vertices = [(-length/2, -width/2, 0), (-length/2, width/2, 0), (length/2, -width/2, 0), (length/2, width/2, 0)]
        if plane == Physics.PLANE_XZ:
            self.vertices = [(-length/2, 0, -width/2), (-length/2, 0, width/2), (length/2, 0, -width/2), (length/2, 0, width/2)]
        if plane == Physics.PLANE_YZ:
            self.vertices = [(0, -length/2, -width/2), (0, -length/2, width/2), (0, length/2, -width/2), (0, length/2, width/2)]

        self.faceVertexIndices = [(0, 1, 2, 3)]

class MeshPlane(MeshObject):
    def __init__(self, position, length, width, plane=Physics.PLANE_XZ, meshFrequency=1, borderColor=GREY, fillColor=GREY):
        super().__init__(position, borderColor=borderColor, fillColor=fillColor)
        self.length = length
        self.width = width
        self.plane = plane
        self.vertices = []
        self.faceVertexIndices = []

        xAmount = int(meshFrequency*length)+1
        yAmount = int(meshFrequency*width)+1

        if plane == Physics.PLANE_XY:
            for x in range(xAmount):
                self.vertices.append((-length/2+x/meshFrequency, -width/2, 0))
                self.vertices.append((-length/2+x/meshFrequency, width/2, 0))
                self.faceVertexIndices.append((2*x,2*x+1))
            for y in range(yAmount):
                self.vertices.append((-length/2, -width/2+y/meshFrequency, 0))
                self.vertices.append((length/2, -width/2+y/meshFrequency, 0))
                self.faceVertexIndices.append((2*xAmount+2*y,2*xAmount+2*y+1))
        if plane == Physics.PLANE_XZ:
            for x in range(xAmount):
                self.vertices.append((-length/2+x/meshFrequency, 0, -width/2))
                self.vertices.append((-length/2+x/meshFrequency, 0, width/2))
                self.faceVertexIndices.append((2*x,2*x+1))
            for y in range(yAmount):
                self.vertices.append((-length/2, 0, -width/2+y/meshFrequency))
                self.vertices.append((length/2, 0, -width/2+y/meshFrequency))
                self.faceVertexIndices.append((2*xAmount+2*y,2*xAmount+2*y+1))
        if plane == Physics.PLANE_YZ:
            for x in range(xAmount):
                self.vertices.append((0, -length/2+x/meshFrequency, -width/2))
                self.vertices.append((0, -length/2+x/meshFrequency, width/2))
                self.faceVertexIndices.append((2*x,2*x+1))
            for y in range(yAmount):
                self.vertices.append((0, -length/2, -width/2+y/meshFrequency))
                self.vertices.append((0, length/2, -width/2+y/meshFrequency))
                self.faceVertexIndices.append((2*xAmount+2*y,2*xAmount+2*y+1))

    def getFaceVerticesRelPos(self):
        arr = [ 0, \
                2*int(self.meshFrequency*self.length), \
                int(self.meshFrequency*self.length)+2*int(self.meshFrequency*self.width), \
                int(self.meshFrequency*self.length)+2*int(self.meshFrequency*self.width)+1 ]
        return [[[pos for j,pos in enumerate(self.vertices[pIndex])] for pIndex in face] for face in arr]


class PixelObject(MeshObject):
    def __init__(self, position=[0,0,0]):
        super().__init__(position)
        self.vertices = [[0, 0, 0], [0.001, 0, 0], [0, 0, 0.001]]
        self.faceVertexIndices = [[0, 1, 2]]
