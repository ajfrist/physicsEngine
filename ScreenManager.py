#ScreenManager.py

import pygame
import math
from Objects import *
from Constants import *

pygame.init()
surface = None
camera = None
font = pygame.font.SysFont("Arial", 15)

def setMainCamera(newCamera):
    global camera
    camera = newCamera

def getMainCamera():
    return camera

def updateScreen(asurface, objectList):
    global surface
    surface = asurface
    if camera != None:
        for obj in objectList:
            drawObject(asurface, obj, obj.borderColor)

def drawObject(surface, obj, color=GREY):
    for face in obj.getFaceVertexIndices():
        drawFace(surface, obj.getFaceVerticesPos(faceGiven=face), color)

def drawFace(surface, verticesWorld, color=GREY, borderWidth=2):
    pixelList = [getPixelFromAngle(getAngleAtPoint(vertex)) for vertex in verticesWorld]
    
    #verify no pixels drawn that are behind camera or stretching across screen
    if None not in pixelList:
        pygame.draw.polygon(surface, color, pixelList, width=borderWidth)

#point as (x, y, z), return angle from camera as (thetaX, thetaY)
def getAngleAtPoint(point):
    point = [point[0]-camera.position[0], point[1]-camera.position[1], point[2]-camera.position[2]]
    thetaX = 0
    smallerHorz = 0
    
    if abs(point[2]) - abs(point[0]) >= 0:
        vertDistance = math.sqrt(point[2]**2 + point[1]**2)
        smallerHorz = point[0]
        largerHorz = point[2]
    else:
        vertDistance = math.sqrt(point[0]**2 + point[1]**2)
        smallerHorz = point[2]
        largerHorz = point[0]
    if vertDistance != 0:
        # thetaX = math.atan(smallerHorz / vertDistance)
        thetaX = math.atan(smallerHorz / largerHorz)
    elif smallerHorz != 0:
        thetaX = sign(smallerHorz) * math.pi/2
    

    # project object in all 360 degrees horzontal
    # if x distance is smallest
    if smallerHorz == point[0]:
        # z displaced is positive (object is to the front in world space)
        if sign(point[2]) > 0:
            thetaX = thetaX
        # z displaced is negative (object is to the behind in world space)
        if sign(point[2]) < 0:
            thetaX += math.pi

    # if z distance is smallest
    else:
        # x displaced is positive (object is to the right in world space)
        if sign(point[0]) > 0:
            thetaX = math.pi/2-thetaX
            
        # x displaced is negative (object is to the left in world space)
        elif sign(point[0]) < 0:
            thetaX = -math.pi/2-thetaX

    horzDistance = math.sqrt(point[0]**2 + point[2]**2)
    objHorzAngle = thetaX
    point[0] += horzDistance * (math.sin(objHorzAngle - toRad(camera.rotation[0])) - math.sin(objHorzAngle))
    point[2] += horzDistance * (math.cos(objHorzAngle - toRad(camera.rotation[0])) - math.cos(objHorzAngle))
    relForward = horzDistance * math.cos(objHorzAngle - toRad(camera.rotation[0]))

    relVertDistance = math.sqrt(relForward**2 + point[1]**2)
    objVertAngle = math.atan(point[1] / abs(relForward))
    if sign(relForward) < 0:
        objVertAngle += 2 * (-math.pi/2 - objVertAngle)
    point[1] += relVertDistance * (math.sin(objVertAngle-toRad(camera.rotation[1])) - math.sin(objVertAngle))
    point[2] += relVertDistance * (math.cos(objVertAngle-toRad(camera.rotation[1])) - math.cos(objVertAngle))
    # print(point)
    if point[2] != 0:
        relativeThetaX = math.atan(point[0] / point[2])
        relativeThetaY = math.atan(point[1] / point[2]) 
    else:
        relativeThetaX = 0
        relativeThetaY = sign(point[1]) * math.pi/2

    if point[2] < 0:
        relativeThetaX = math.pi/2 - relativeThetaX
        if relativeThetaX != 0:
            relativeThetaX *= sign(relativeThetaX)

    if relativeThetaY > toRad(88) or point[2] < 0:
        return None
    return [relativeThetaX, relativeThetaY]

#returns screen pixel (x, y) based on angle from camera (thetaX, thetaY)
def getPixelFromAngle(angleFromCamera):
    if angleFromCamera == None:
        return None
    # print(angleFromCamera)
    dX = LOADING_RADIUS * math.sin(angleFromCamera[0])
    dY = LOADING_RADIUS * math.sin(angleFromCamera[1])
    return (int(DISPLAY_WIDTH/2 + dX), -int(dY - HORIZON_OFFSET))

#returns base angle with 0 <= x < 360
def normalizeAngleDeg(angle):
    while angle >= 360:
            angle -= 360
    while angle < 0:
        angle += 360
    return angle

#returns base angle with 0 <= x < 2pi
def normalizeAngleRad(angle):
    while angle >= 2*math.pi:
            angle -= 2*math.pi
    while angle < 0:
        angle += 2*math.pi
    return angle

#returns shortest difference between angles
def angleSubtractDeg(target, source):
    a = target - source
    if a > 180:
        a -= 360
    if a < -180:
        a += 360
    return a

def angleSubtractRad(target, source):
    a = target - source
    if a > math.pi:
        a -= 2*math.pi
    if a < -math.pi:
        a += 2*math.pi
    return a

def toRad(a):
    return a * math.pi / 180

def toDeg(a):
    return a * 180 / math.pi

#returns 1 if num > 0, -1 if num < 0
def sign(num):
    if num == 0:
        return 1
    return num / abs(num)

def textToScreen(textString, pos):
    global surface
    if surface == None:
        return
    text1 = font.render(textString, False, (0, 0, 255))
    rect1 = text1.get_rect()
    rect1.left = pos[0]
    rect1.top = pos[1]
    surface.blit(text1, rect1)

