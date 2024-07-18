#Rigidbody.py

from Vector import Vector3
from Physics import Equation
import Physics
import PhysicsManager

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
    # print("kkkkkkkkkkkkk")
    for c in collisions:
        # input(c.collisionPoints)
        #obtain average collision point
        point = Vector3(0, 0, 0)
        for p in c.collisionPoints:
            # print(p)
            point += Vector3(p)
        point = point / len(c.collisionPoints)
        # print(f"  {point}")

        #if body is movable, determine normal force vector and apply perfect elastic velocity change
        if c.body1.mass != math.inf and c.body2.mass != math.inf:
            normalAngle1 = -(point - c.body1.position)
            normalAngle2 = -(point - c.body2.position)
            normalAngle1.normalizeSelf()
            normalAngle2.normalizeSelf()
            print(normalAngle1)
            # print(normalAngle2)
            # print("**")
            # TODO - potential for rapid/incorrect acceleration on collisions (non-perfect elastic ones?)
                # due to repeat collision calls - solution: move object along normal vect some dist to remove collision before next call
            for i in range(3):
                vel1 = c.body1.velocity[i]
                vel2 = c.body2.velocity[i]
                vf1 = (c.body1.mass*vel1+c.body2.mass*vel2-c.body2.mass*(vel1-vel2))/(c.body1.mass+c.body2.mass)
                # vf1 = (c.body1.mass*vel1+c.body2.mass*vel2-c.body2.mass*(vel2-vel1))/(c.body1.mass+c.body2.mass)
                vf2 = vel1-vel2+vf1
                # print(c.body1.velocity[i],c.body2.velocity[i])
                # print(vf1, vf2)
                c.body1.velocity[i] = vf1 #* abs(normalAngle1[i])
                c.body2.velocity[i] = vf2 #* abs(normalAngle2[i])
                # print(c.body1.velocity[i],c.body2.velocity[i])
            # print(c.bod y2.velocity)







#called by main physics thread to check for rigidbody collisions on all objects
#returns True if any collisions detected, otherwise False
def checkAllCollisions():
    if len(allBodies) < 2:
        return None
    # print("1")
    collisions = []
    for i in range(len(allBodies)):
        for j in range(i+1, len(allBodies)):
            dist = math.sqrt(abs(allBodies[i].position[0]-allBodies[j].position[0])**2 + \
                abs(allBodies[i].position[1]-allBodies[j].position[1])**2 + \
                abs(allBodies[i].position[2]-allBodies[j].position[2])**2)
            # print("2")
            #bodies are far enough apart that they can't be touching
            # print(dist)
            if dist > allBodies[i].maxDistanceFromCenter + allBodies[j].maxDistanceFromCenter:
                return None
            # print("3")


            #deprecated - unlikely to implement, even the correct version
            #fix equation - calculate normals for faces

            # #bodies are closer enough that they have to be touching
            # if dist < allBodies[i].minDistanceFromCenter + allBodies[j].minDistanceFromCenter:
            #     return True
            # print("4")
            #check standard collision
            collisionPoints = checkCollision(allBodies[i], allBodies[j])
            # print("5")
            if collisionPoints != None:
                # print("_5")
                # print(collisionPoints)
                coll = Collision(allBodies[i],allBodies[j],collisionPoints)
                collisions.append(coll)
    # print("6")
    if len(collisions) > 0:
        return collisions
    return None

#check collisions between two rigidbodies
#returns list of any collisions detected, otherwise None
def checkCollision(body1, body2):
    # print("rann")
    #TODO - benchmark this vs implementing FACE object with face.(min/max)DistFromCenter 
        #     and auto not calculate collision if face is too far away from other face
    trueHits = []
    hitPoints= [[],[]]
    for a,faceBounds in enumerate(body1.boundaryEquations):
        for b,line in enumerate(faceBounds):
            for c,b2face in enumerate(body2.boundaryEquations):
                #if on cube1's right face and cube2's left side
                # if a==2 and c == 0:
                #     print("*")
                #     print(line.endpoints)
                print(f"{a}, {b}, {c}")
                val = [a,b,c,0]
                results, indices = isLineInBoundary(line, b2face, Physics.PLANE_XY,val)
                # print("v1: "+str(segment is not None))
                if results == None:
                    continue
                for result, b2Index in zip(results, indices):
                    #single point intersect on XY plane
                    if result != None and isinstance(result, tuple):
                        print("point1")
                        print(f"   {a}, {b}, {c}, {b2Index}")
                        l1 = Physics.generateEquation(line.endpoints[0], line.endpoints[1], Physics.PLANE_XZ)
                        # print("-")
                        l2 = Physics.generateEquation(b2face[b2Index].endpoints[0], b2face[b2Index].endpoints[1], Physics.PLANE_XZ)
                        # print("-")
                        z1 = l1.evaluate(result[0])
                        # print("-")
                        z2 = l2.evaluate(result[0])
                        # print("-")
                        # print(l1.endpoints)
                        # print(l1.coefficient)
                        # print(l1.intercept)
                        # print(l2.endpoints)
                        # print(l2.coefficient)
                        # print(l2.intercept)
                        # print(z1)
                        # print(z2)
                        # print(abs(z1-z2))
                        # print(abs(z1-z2)>0.00001)
                        # print("<")
                        #single point intersect on XZ plane
                        if abs(z1 - z2) <= 0.00001:
                            print("point2")
                            print(str((result[0],result[1],z1)))
                            trueHits.append((a,b,c,b2Index))
                            hitPoints[0].append((result[0],result[1],z1))
                            hitPoints[1].append((a,b,c,b2Index))
                            #return True

                    elif result != None and isinstance(result, list) and isinstance(result[1], bool):
                        result[0] = Physics.generateEquation(result[0].endpoints[0], result[0].endpoints[1], Physics.PLANE_XZ)
                        results2, indices2 = isLineInBoundary(result[0], b2face, Physics.PLANE_XZ,val)
                        if results2 == None:
                            continue
                        for result2, b2Index2 in zip(results2, indices2):
                            if result2 != None and isinstance(result2, list) and isinstance(result2[1], bool):
                                result2[0] = Physics.generateEquation(result2[0].endpoints[0], result2[0].endpoints[1], Physics.PLANE_YZ)
                                results3, indices3 = isLineInBoundary(result2[0], b2face, Physics.PLANE_YZ, val)
                                if results3 == None:
                                    continue
                                for result3, b2Index3 in zip(results3, indices3):
                                    if result3 != None and isinstance(result3, list) and isinstance(result3[1], bool):
                                        hitPoints[0].append(result3[0])
                                        hitPoints[1].append("OPEN COLLISION")
                                

                    #multipoint/segment intersection on XY plane
                    # elif result != None and isinstance(result, Physics.Equation):
                    elif result != None and isinstance(result, list):
                        print(b2Index)
                        print("seg1")
                        print(f"   {a}, {b}, {c}, {b2Index}")
                        print(result[0].endpoints)
                        end1 = result[0].endpoints[0]
                        end2 = result[0].endpoints[1]
                        # print(end1,end2)
                        if result[0].plane != Physics.PLANE_XZ:
                            result[0] = Physics.generateEquation(end1, end2, Physics.PLANE_XZ)
                        results2, indices2 = isLineInBoundary(result[0], [result[1]], Physics.PLANE_XZ,val)
                        # print("v2: "+str(segment2 is not None))
                        # intersection on XZ plane
                        if results2 == None:
                            continue
                        # print(results2)
                        for segment2, b2Index2 in zip(results2, indices2):
                            # print(segment.endpoints)
                            # print(segment.coefficient)
                            # print(segment.intercept)
                            # print(segment2.endpoints)
                            # print(segment2.coefficient)
                            # print(segment2.intercept)                        
                            # print(segment2 != None)
                            # print("<")
                            # if segment2 != None and b2Index == b2Index2:
                            if segment2 != None:
                                print(b2Index2)
                                print("seg2")
                                # print("alkLKJDFHLSDKJFHLSKJFHLKJSDHFLK")
                                trueHits.append((a,b,c,b2Index2))
                                if isinstance(segment2, tuple):
                                    #TODO - y value here is just not correct, not accounting for anything other than vertical line;
                                       #must calculate actual y value at intersection
                                    hitPoints[0].append((segment2[0],segment.endpoints[0][1],segment2[1]))
                                    print(str((segment2[0],segment.endpoints[0][1],segment2[1])))
                                else:
                                    if abs(segment2[0].endpoints[0][0]-segment2[0].endpoints[1][0]) <= 0.00001 and \
                                            abs(segment2[0].endpoints[0][1]-segment2[0].endpoints[1][1]) <= 0.00001 and \
                                            abs(segment2[0].endpoints[0][2]-segment2[0].endpoints[1][2]) <= 0.00001:
                                        # hitPoints[0].append(segment2.endpoints[0])
                                        hitPoints[0].append((segment2[0].endpoints[0][0],segment2[0].endpoints[0][1],segment2[0].endpoints[0][2]))
                                        print(segment2[0].endpoints)
                                    else:
                                        # hitPoints[0].append(segment2)
                                        hitPoints[0].append((segment2[0].endpoints[0][0],segment2[0].endpoints[0][1],segment2[0].endpoints[0][2]))
                                        hitPoints[0].append((segment2[0].endpoints[1][0],segment2[0].endpoints[1][1],segment2[0].endpoints[1][2]))
                                        hitPoints[1].append((a,b,c,b2Index2))
                                        print(segment2[0].endpoints)

                                hitPoints[1].append((a,b,c,b2Index2))
                                #return True
    print("HIT POINTS:")
    for i in range(len(hitPoints[0])):
        if not hitPoints[0][i] in hitPoints[0][0:i]:
            print(hitPoints[0][i],end="    ,   ")
            print(hitPoints[1][i])
            if isinstance(hitPoints[0][i], Physics.Equation):

                print("    ",end="")
                print(hitPoints[0][i].endpoints)
    print("<")
    # print("TRUE HITS:")
    # for x in trueHits:
    #     print(x)
    # print("<")
    l=[]
    i=1
    while (i < len(hitPoints[0])):
        for p in hitPoints[0][0:i]:
            if abs(hitPoints[0][i][0] - p[0]) <= 0.005 and \
                abs(hitPoints[0][i][1] - p[1]) <= 0.005 and \
                abs(hitPoints[0][i][2] - p[2]) <= 0.005:
                del(hitPoints[0][i])
                i-=1
                break
        i+=1

    # i=0
    # while (i <= len(hitPoints[0])):
    #     # if hitPoints[0][i] in hitPoints[0][0:i]:
    #     for p in hitPoints[0:i]:
    #         if abs(hitPoints[0][i][0] - p[0]) <= 0.00001 and \
    #                 abs(hitPoints[0][i][1] - p[1]) <= 0.00001 and \
    #                 abs(hitPoints[0][i][2] - p[2]) <= 0.00001:
    #             del(hitPoints[0][i])
    #         else:
    #             i+=1


    #     if hitPoints[0][i][0]
    #         del(hitPoints[0][i])
    #     else:
    #         i+=1
    # i=0
    # while (i < len(hitPoints[0])):
    #     # if hitPoints[0][i] in hitPoints[0][0:i]:
    #     if hitPoints[0][i][0]
    #         del(hitPoints[0][i])
    #     else:
    #         i+=1
    if len(trueHits) != 0:
        return hitPoints[0]
    return None
        

#generate equations defining the points bounding a face
#(FVPs) faceVerticesPos - ex:
#   [(0, 0, 0), (1, 1, 1), (2, 2, 2)]   <- face 1
def generateFaceBoundaryEquations(FVPs, averagePoint, plane, parent=None):
    lis = []
    
    for i in range(len(FVPs)):
        if i+1 == len(FVPs):
            eq = Physics.generateEquation(FVPs[i], FVPs[0], plane, parent)
        else:
            eq = Physics.generateEquation(FVPs[i], FVPs[i+1], plane, parent)
        # print(eq.endpoints)

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


#TODO - what if intersecting line never actually intersects a boundary, but is within region?
#divide by zero protection

#TODO - function returns first segment of intersection - missing out on other potential intersections in the same face

#determines if a line interests with an area
#returns Equation defining the first segment of intersection,
#       or (x,y) tuple of the singular intersection point
#       otherwise returns None
#line - Equation
#boundaries - list of Equation objects
#plane - PLANE_XY
def isLineInBoundary(line, boundaries, plane,val):
    #input("halt")
    results = []
    indices = []
    if plane == Physics.PLANE_XY:
        for a,boundary in enumerate(boundaries):
            val[3]=a
            # print(boundary.endpoints)
            # print(f"     ____{val}")
            # print("XY")
            # print(line.endpoints)
            # print(boundary.endpoints)
            #potentially one intersection - both lines are only points
            if line.coefficient == None and boundary.coefficient == None:
                # print("_1")
                if abs(line.endpoints[0][0]-boundary.endpoints[0][0]) <= 0.00001 and \
                        abs(line.endpoints[0][1]-boundary.endpoints[0][1]) <= 0.00001:
                    print("_ _1")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (line.endpoints[0][0], line.endpoints[0][1]), a
                    results.append((line.endpoints[0][0], line.endpoints[0][1]))
                    indices.append(a)
                    # print((val[0],val[1],val[2],val[3]))
            #potentially one intersection - one line is a point, other is line
            elif line.coefficient == None:
                # print("_2")
                x1 = line.endpoints[0][0]
                y2 = boundary.evaluate(x=x1)
                if abs(line.endpoints[0][1]-y2) <= 0.00001 and \
                        x1 <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x1 >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        y2 <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        y2 >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]):
                    print("_ _2")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # print((val[0],val[1],val[2],val[3]))
                    # return (line.endpoints[0][0], line.endpoints[0][1]), a
                    results.append((line.endpoints[0][0], line.endpoints[0][1]))
                    indices.append(a)
            #potentially one intersection - one line is line, other line is a point
            elif boundary.coefficient == None:
                # print("_3")
                x1 = boundary.endpoints[0][0]
                y2 = line.evaluate(x=x1)
                if abs(boundary.endpoints[0][1]-y2) <= 0.00001 and \
                        x1 <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x1 >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        y2 <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                        y2 >= min(line.endpoints[0][1], line.endpoints[1][1]):
                    print("_ _3")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # print((val[0],val[1],val[2],val[3]))
                    # return (boundary.endpoints[0][0], boundary.endpoints[0][1]), a
                    results.append((boundary.endpoints[0][0], boundary.endpoints[0][1]))
                    indices.append(a)
            #lines parallel vertical, potential segment intersection
            elif line.coefficient == math.inf and boundary.coefficient == math.inf: 
                print("_ _41")
                if abs(line.endpoints[0][0] - boundary.endpoints[0][0]) <= 0.00001:
                    allHits.append((val[0],val[1],val[2],val[3]))
                    lis = sortPointsByAxis([line.endpoints[0], line.endpoints[1], boundary.endpoints[0], boundary.endpoints[1]], Physics.AXIS_Y)
                    # return Physics.generateEquation(lis[1], lis[2], plane), a

                    # results.append(Physics.generateEquation(lis[1], lis[2], plane))

                    #line & boundary equations in XZ plane
                    l2 = Physics.generateEquation(line.endpoints[0],line.endpoints[1], Physics.PLANE_XZ)
                    b2 = Physics.generateEquation(boundary.endpoints[0],boundary.endpoints[1], Physics.PLANE_XZ)

                    #calculate innermost two points using the inner x bounds
                    lineZ1 = l2.evaluate(x=lis[1][0])
                    lineZ2 = b2.evaluate(x=lis[1][0])
                    boundaryZ1 = l2.evaluate(x=lis[2][0])
                    boundaryZ2 = b2.evaluate(x=lis[2][0])
                    returnSeg1 = Physics.generateEquation((line.endpoints[0][0],lis[1][1],lineZ1), (line.endpoints[0][0],lis[2][1],lineZ2), Physics.PLANE_XZ)
                    returnSeg2 = Physics.generateEquation((line.endpoints[0][0],lis[1][1],boundaryZ1), (line.endpoints[0][0],lis[2][1],boundaryZ2), Physics.PLANE_XZ)
                    results.append([returnSeg1,returnSeg2])
                    indices.append(a)
                    # print((val[0],val[1],val[2],val[3]))
            #one intersection - vertical line and regular line 
            elif line.coefficient == math.inf:
                # print("_ _42")
                x = line.endpoints[0][0]
                y = boundary.coefficient * x + boundary.intercept
                if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        y <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        y >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                        y >= min(line.endpoints[0][1], line.endpoints[1][1]):
                    print("_ _ _421")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (x, y), a
                    results.append((x, y))
                    indices.append(a)
                    # print((val[0],val[1],val[2],val[3]))
            #one intersection - regular line and vertical line
            elif boundary.coefficient == math.inf:
                # print("_ _43")
                x = boundary.endpoints[0][0]
                y = line.coefficient * x + line.intercept
                # print(">")
                # print(x)
                # print(y)
                # print(line.coefficient)
                # print(line.intercept)
                # print(line.endpoints)
                # print(boundary.endpoints)
                # print(x <= max(line.endpoints[0][0], line.endpoints[1][0]))
                # print(x >= min(line.endpoints[0][0], line.endpoints[1][0]))
                # print(y <= max(line.endpoints[0][1], line.endpoints[1][1]))
                # print(y >= min(line.endpoints[0][1], line.endpoints[1][1]))
                # print("<")
                if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        y <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        y >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                        y >= min(line.endpoints[0][1], line.endpoints[1][1]):
                    print("_ _ _431")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (x, y), a
                    results.append((x, y))
                    indices.append(a)
                    # print((val[0],val[1],val[2],val[3]))
            #lines intersect once
            elif abs(line.coefficient - boundary.coefficient) > 0.00001:
                # print("_4")
                # if line.coefficient == math.inf and boundary.coefficient == math.inf:
                #     print("_ _41")
                #     return None
                # elif line.coefficient == math.inf:
                #     print("_ _42")
                #     return (line.endpoints[0][0], boundary.coefficient * line.endpoints[0][0] + boundary.intercept)
                # elif boundary.coefficient == math.inf:
                #     print("_ _43")
                #     return (boundary.endpoints[0][0], line.coefficient * boundary.endpoints[0][0] + line.intercept)
                # else:
                #     print("_ _44")
                #     x = (boundary.intercept - line.intercept) / (line.coefficient - boundary.coefficient)
                #     y = boundary.coefficient * x + boundary.intercept
                # print("_ _44")
                x = (boundary.intercept - line.intercept) / (line.coefficient - boundary.coefficient)
                y = boundary.coefficient * x + boundary.intercept
                #ensure intersection is within bounds                
                if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        y <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        y >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                        y >= min(line.endpoints[0][1], line.endpoints[1][1]):
                    print("_ _45")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (x, y), a
                    results.append((x, y))
                    indices.append(a)
                    # print((val[0],val[1],val[2],val[3]))
            #lines parallel, no intersect
            # elif abs(boundary.intercept - line.intercept) > 0.00001:
            #     pass
            #     print("_5")     
                # return None, a
                # print((val[0],val[1],val[2],val[3]))
            #lines parallel, whole intersect, return smaller (innermost) segment of the two (among the four endpoints)
            else:
                # print(abs(boundary.coefficient - line.coefficient))
                # print("_6")
                # print(line.coefficient, boundary.coefficient)
                #if at least one endpoint from line is within segment ends of boundary
                if (line.endpoints[0][0] <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[0][0] >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[0][1] <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        line.endpoints[0][1] >= min(boundary.endpoints[0][1], boundary.endpoints[1][1])) or \
                       (line.endpoints[1][0] <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[1][0] >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[1][1] <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                        line.endpoints[1][1] >= min(boundary.endpoints[0][1], boundary.endpoints[1][1])):
                    print("_ _6")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    lis = sortPointsByAxis([line.endpoints[0], line.endpoints[1], boundary.endpoints[0], boundary.endpoints[1]], Physics.AXIS_X)
                    # return Physics.generateEquation(lis[1], lis[2], plane), a

                    # results.append(Physics.generateEquation(lis[1], lis[2], plane))

                    # order = []
                    # for p in lis:
                    #     order.append(lis.index(p))


                    # if line.endpoints[1]

                    # returnSeg1 = 
                    # returnSeg2

                    # yVal1 = 
                    # #line endpoints are on outside, boundary endpoints on inside
                    # if (line.endpoints[0] == lis[0] or line.endpoints[0] == lis[3]) and \
                    #         (line.endpoints[1] == lis[0] or line.endpoints[1] == lis[3]):
                    #     returnSeg1 = Physics.generateEquation((boundary.endpoints[0][0],line.endpoints[0][1],line.endpoints[0][2]), (boundary.endpoints[1][0],line.endpoints[1][1],line.endpoints[1][2]), plane)
                    #     returnSeg2 = Physics.generateEquation(boundary.endpoints[0], boundary.endpoints[1],plane)
                    # #line endpoints are on inside, boundary endpoints on outside 
                    # elif (line.endpoints[0] == lis[1] or line.endpoints[0] == lis[2]) and \
                    #         (line.endpoints[1] == lis[1] or line.endpoints[1] == lis[2]):
                    #     returnSeg1 = Physics.generateEquation(line.endpoints[0], line.endpoints[1],plane)
                    #     returnSeg2 = Physics.generateEquation((line.endpoints[0][0],boundary.endpoints[0][1],boundary.endpoints[0][2]), (line.endpoints[1][0],boundary.endpoints[1][1],boundary.endpoints[1][2]), plane)
                    # #line endpoints are on left and mid-right side, boundary endpoints are on mid-left and right side
                    # elif (line.endpoints[0] == lis[0] or line.endpoints[0] == lis[2]) and \
                    #         (line.endpoints[1] == lis[0] or line.endpoints[1] == lis[2]):
                    #     if line.endpoints[0] == lis[0]:
                    #         returnSeg1 = Physics.generateEquation((boundary.endpoints[0][0],yVal1,zVal1), line.endpoints[1], plane)
                    #         if boundary.endpoints[0] == lis[1]:
                    #             returnSeg2 = Physics.generateEquation(boundary.endpoints[0], (line.endpoints[1][0],boundary.endpoints[1][1],boundary.endpoints[1][2]), plane)
                    #         else:

                    # #line endpoints are on mid-left and right side, boundary endpoints are on left and mid-right side
                    # elif (line.endpoints[0] == lis[1] or line.endpoints[0] == lis[3]) and \
                    #         (line.endpoints[1] == lis[1] or line.endpoints[1] == lis[3]):

                    #line & boundary equations in XZ plane
                    l2 = Physics.generateEquation(line.endpoints[0],line.endpoints[1], Physics.PLANE_XZ)
                    b2 = Physics.generateEquation(boundary.endpoints[0],boundary.endpoints[1], Physics.PLANE_XZ)

                    #calculate innermost two points using the inner x bounds
                    lineY1 = line.evaluate(x=lis[1][0])
                    lineZ1 = l2.evaluate(x=lis[1][0])
                    lineY2 = line.evaluate(x=lis[2][0])
                    lineZ2 = l2.evaluate(x=lis[2][0])
                    boundaryY1 = boundary.evaluate(x=lis[1][0])
                    boundaryZ1 = b2.evaluate(x=lis[1][0])
                    boundaryY2 = boundary.evaluate(x=lis[2][0])
                    boundaryZ2 = b2.evaluate(x=lis[2][0])
                    returnSeg1 = Physics.generateEquation((lis[1][0],lineY1,lineZ1), (lis[2][0],lineY2,lineZ2), Physics.PLANE_XZ)
                    returnSeg2 = Physics.generateEquation((lis[1][0],boundaryY1,boundaryZ1), (lis[2][0],boundaryY2,boundaryZ2), Physics.PLANE_XZ)
                    results.append([returnSeg1,returnSeg2])

                    print("**")
                    print(l2.endpoints, l2.coefficient, l2.intercept)
                    print(b2.endpoints, b2.coefficient, b2.intercept)
                    print(returnSeg1.endpoints)
                    print(returnSeg2.endpoints)
                    # print(id(returnSeg1), repr(returnSeg1))
                    # print(id(returnSeg2), repr(returnSeg2))
                    print("**")

                    indices.append(a)
                    # print((val[0],val[1],val[2],val[3]))
                #boundary sections
                else:
                    input("_6.5")
                    if boundary.evaluate(x=line.endpoints[0][0], y=line.endpoints[0][1]) or \
                            boundary.evaluate(x=line.endpoints[1][0], y=line.endpoints[1][1]):
                        x = (boundary.intercept - line.intercept) / (line.coefficient - boundary.coefficient)
                        y = boundary.coefficient * x + boundary.intercept
                        #ensure intersection is within bounds                
                        if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                                x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                                y <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                                y >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                                x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                                x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                                y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                                y >= min(line.endpoints[0][1], line.endpoints[1][1]):
                            results.append([(x,y), True])

            # print("--")
    elif plane == Physics.PLANE_XZ:
        for a,boundary in enumerate(boundaries):
            if boundary.plane != Physics.PLANE_XZ:
                boundary = Physics.generateEquation(boundary.endpoints[0], boundary.endpoints[1], Physics.PLANE_XZ)
            # print(f", {a}")
            # print("XZ")
            # print(line.endpoints)
            # print(boundary.endpoints)
            val[3]=a
            #one intersection - both lines are only points
            if line.coefficient == None and boundary.coefficient == None:
                # print("_7")
                if abs(line.endpoints[0][0]-boundary.endpoints[0][0]) <= 0.00001 and \
                        abs(line.endpoints[0][2]-boundary.endpoints[0][2]) <= 0.00001:
                    # print("_ _7")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (line.endpoints[0][0], line.endpoints[0][2]), a
                    results.append((line.endpoints[0][0], line.endpoints[0][2]))
                    indices.append(a)
            #one intersection - one line is a point, other is line
            elif line.coefficient == None:
                # print("_8")
                x1 = line.endpoints[0][0]
                z2 = boundary.coefficient
                z2 = boundary.evaluate(x=x1)
                if abs(line.endpoints[0][2]-z2) <= 0.00001 and \
                        x1 <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x1 >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        z2 <= max(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        z2 >= min(boundary.endpoints[0][2], boundary.endpoints[1][2]):
                    # print("_ _8")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (line.endpoints[0][0], line.endpoints[0][2]), a
                    results.append((line.endpoints[0][0], line.endpoints[0][2]))
                    indices.append(a)
            #one intersection - one line is line, other line is a point
            elif boundary.coefficient == None:
                # print("_9")
                x1 = boundary.endpoints[0][0]
                z2 = line.evaluate(x=x1)
                if abs(boundary.endpoints[0][2]-z2) <= 0.00001 and \
                        x1 <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x1 >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        z2 <= max(line.endpoints[0][2], line.endpoints[1][2]) and \
                        z2 >= min(line.endpoints[0][2], line.endpoints[1][2]):
                    # print("_ _9")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (boundary.endpoints[0][0], boundary.endpoints[0][2]), a
                    results.append((boundary.endpoints[0][0], boundary.endpoints[0][2]))
                    indices.append(a)
            #lines parallel vertical
            elif line.coefficient == math.inf and boundary.coefficient == math.inf: 
                # print("_ _101")
                if abs(line.endpoints[0][0] - boundary.endpoints[0][0]) <= 0.00001:
                    allHits.append((val[0],val[1],val[2],val[3]))
                    lis = sortPointsByAxis([line.endpoints[0], line.endpoints[1], boundary.endpoints[0], boundary.endpoints[1]], Physics.AXIS_Y)
                    # return Physics.generateEquation(lis[1], lis[2], plane), a

                    # results.append(Physics.generateEquation(lis[1], lis[2], plane))
                    
                    #line & boundary equations in XZ plane
                    l2 = Physics.generateEquation(line.endpoints[0],line.endpoints[1], Physics.PLANE_XZ)
                    b2 = Physics.generateEquation(boundary.endpoints[0],boundary.endpoints[1], Physics.PLANE_XZ)

                    #calculate innermost two points using the inner x bounds
                    lineZ1 = l2.evaluate(x=lis[1][0])
                    lineZ2 = b2.evaluate(x=lis[1][0])
                    boundaryZ1 = l2.evaluate(x=lis[2][0])
                    boundaryZ2 = b2.evaluate(x=lis[2][0])
                    returnSeg1 = Physics.generateEquation((line.endpoints[0][0],lis[1][1],lineZ1), (line.endpoints[0][0],lis[2][1],lineZ2), Physics.PLANE_XZ)
                    returnSeg2 = Physics.generateEquation((line.endpoints[0][0],lis[1][1],boundaryZ1), (line.endpoints[0][0],lis[2][1],boundaryZ2), Physics.PLANE_XZ)
                    results.append([returnSeg1,returnSeg2])
                    indices.append(a)
            #one intersection - vertical line and regular line 
            elif line.coefficient == math.inf:
                # print("_ _102")
                x = line.endpoints[0][0]
                z = boundary.coefficient * x + boundary.intercept
                if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        z <= max(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        z >= min(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        z <= max(line.endpoints[0][2], line.endpoints[1][2]) and \
                        z >= min(line.endpoints[0][2], line.endpoints[1][2]):
                    # print("_ _ _1021")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (x, z), a
                    results.append((x, z))
                    indices.append(a)
            #one intersection - regular line and vertical line
            elif boundary.coefficient == math.inf:
                # print("_ _103")
                x = boundary.endpoints[0][0]
                z = line.coefficient * x + line.intercept
                if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        z <= max(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        z >= min(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        z <= max(line.endpoints[0][2], line.endpoints[1][2]) and \
                        z >= min(line.endpoints[0][2], line.endpoints[1][2]):
                    # print("_ _ _1031")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (x, z), a
                    results.append((x, z))
                    indices.append(a)
            #lines intersect once
            elif abs(line.coefficient - boundary.coefficient) > 0.00001:
                # print("_10")
                # if line.coefficient == math.inf and boundary.coefficient == math.inf:
                #     print("_ _101")
                #     return None
                # elif line.coefficient == math.inf:
                #     print("_ _102")
                #     return (line.endpoints[0][0], boundary.coefficient * line.endpoints[0][0] + boundary.intercept)
                # elif boundary.coefficient == math.inf:
                #     print("_ _103")
                #     return (boundary.endpoints[0][0], line.coefficient * boundary.endpoints[0][0] + line.intercept)
                # else:
                # print("_ _104")
                x = (boundary.intercept - line.intercept) / (line.coefficient - boundary.coefficient)
                z = boundary.coefficient * x + boundary.intercept
                #ensure intersection is within bounds
                if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        z <= max(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        z >= min(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                        x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                        z <= max(line.endpoints[0][2], line.endpoints[1][2]) and \
                        z >= min(line.endpoints[0][2], line.endpoints[1][2]):
                    # print("_ _105")
                    allHits.append((val[0],val[1],val[2],val[3]))
                    # return (x, z), a
                    results.append((x, z))
                    indices.append(a)
            #lines parallel, no intersect
            elif abs(boundary.intercept - line.intercept) > 0.00001:
                pass
                # print("_11")
                # return None, a
            #lines parallel, whole intersect, return smaller (innermost) segment of the two (among the four endpoints)
            else:
                # print("_12")
                if (line.endpoints[0][0] <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[0][0] >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[0][2] <= max(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        line.endpoints[0][2] >= min(boundary.endpoints[0][2], boundary.endpoints[1][2])) or \
                       (line.endpoints[1][0] <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[1][0] >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                        line.endpoints[1][2] <= max(boundary.endpoints[0][2], boundary.endpoints[1][2]) and \
                        line.endpoints[1][2] >= min(boundary.endpoints[0][2], boundary.endpoints[1][2])):
                    allHits.append((val[0],val[1],val[2],val[3]))
                    lis = sortPointsByAxis([line.endpoints[0], line.endpoints[1], boundary.endpoints[0], boundary.endpoints[1]], Physics.AXIS_X)
                    # return Physics.generateEquation(lis[1], lis[2], plane), a

                    # results.append(Physics.generateEquation(lis[1], lis[2], plane))

                    #line & boundary equations in XY plane
                    l2 = Physics.generateEquation(line.endpoints[0],line.endpoints[1], Physics.PLANE_XY)
                    b2 = Physics.generateEquation(boundary.endpoints[0],boundary.endpoints[1], Physics.PLANE_XY)

                    #calculate innermost two points using the inner x bounds
                    lineY1 = l2.evaluate(x=lis[1][0])
                    lineZ1 = line.evaluate(x=lis[1][0])
                    lineY2 = l2.evaluate(x=lis[2][0])
                    lineZ2 = line.evaluate(x=lis[2][0])
                    boundaryY1 = b2.evaluate(x=lis[1][0])
                    boundaryZ1 = boundary.evaluate(x=lis[1][0])
                    boundaryY2 = b2.evaluate(x=lis[2][0])
                    boundaryZ2 = boundary.evaluate(x=lis[2][0])
                    returnSeg1 = Physics.generateEquation((lis[1][0],lineY1,lineZ1), (lis[2][0],lineY2,lineZ2), Physics.PLANE_XZ)
                    returnSeg2 = Physics.generateEquation((lis[1][0],boundaryY1,boundaryZ1), (lis[2][0],boundaryY2,boundaryZ2), Physics.PLANE_XZ)
                    results.append([returnSeg1,returnSeg2])
                    indices.append(a)
                else:
                    if boundary.evaluate(x=line.endpoints[0][0], y=line.endpoints[0][1]) or \
                            boundary.evaluate(x=line.endpoints[1][0], y=line.endpoints[1][1]):
                        x = (boundary.intercept - line.intercept) / (line.coefficient - boundary.coefficient)
                        y = boundary.coefficient * x + boundary.intercept
                        #ensure intersection is within bounds                
                        if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                                x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                                y <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                                y >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                                x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                                x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                                y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                                y >= min(line.endpoints[0][1], line.endpoints[1][1]):
                            results.append([(x,y), True])
    elif plane == Physics.PLANE_YZ:
        if boundary.evaluate(x=line.endpoints[0][0], y=line.endpoints[0][1]) or \
                boundary.evaluate(x=line.endpoints[1][0], y=line.endpoints[1][1]):
            x = (boundary.intercept - line.intercept) / (line.coefficient - boundary.coefficient)
            y = boundary.coefficient * x + boundary.intercept
            #ensure intersection is within bounds                
            if x <= max(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                    x >= min(boundary.endpoints[0][0], boundary.endpoints[1][0]) and \
                    y <= max(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                    y >= min(boundary.endpoints[0][1], boundary.endpoints[1][1]) and \
                    x <= max(line.endpoints[0][0], line.endpoints[1][0]) and \
                    x >= min(line.endpoints[0][0], line.endpoints[1][0]) and \
                    y <= max(line.endpoints[0][1], line.endpoints[1][1]) and \
                    y >= min(line.endpoints[0][1], line.endpoints[1][1]):
                results.append([(x,y), True])
            # print("---")
    if len(results) != 0:
        return results, indices
    return None, None


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
                 airResistance=0,
                 gravity=0):
        self.parent = parent
        if position is None:
            self.position = Vector3(parent.position)
        else:
            self.position = Vector3(position)
        self.velocity = velocity
        self.acceleration = acceleration
        self.facesVerticesPos = parent.getFaceVerticesRelPos()
        self.boundaryEquations = []
        for face in parent.getFaceVerticesPos():
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

class Collision:
    def __init__(self, body1, body2, collisionPoints):
        self.body1 = body1
        self.body2 = body2
        self.collisionPoints = collisionPoints




        

    
        
