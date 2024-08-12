#PhysicsManager.py

from Vector import Vector3


ALL_RIGIDBODIES = 100

activeForces = []
bodies = []

#add a new force to a rigidbody
def addForce(vector, duration, target):
    if isinstance(target, Rigidbody):
        activeForces.append(Force(vector, duration, target))
    elif isinstance(target, Object):
        activeForces.append(Force(vector, duration, target))
    else:
        raise TypeError("Force must be applied to a Rigidbody type target.")

def applyActiveForces():
    for force in activeForces:
        force.applyForce()

#called by main physics thread - update positions of all physics objects 
def update(deltaTime):
    for body in bodies:
        body.velocity += body.acceleration * deltaTime
        body.parent.move(body.velocity * deltaTime * (1-body.airResistance))
        body.parent.rotateWorld(body.rotationVelocity * deltaTime * (1-body.airResistance))

def addRigidbody(body):
    bodies.append(body)

def getActiveForces():
    return activeForces
