#main.py

import sys
import math
import pygame
import numpy as np

from Objects import *
from Camera import Camera
import ScreenManager as SM
from Constants import *
import Controls
from Vector import Vector3
import Rigidbody
import PhysicsManager
import Physics

def main():
    pygame.init()

    screen = pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT))
    pygame.display.set_caption("Simulator")
    Clock = pygame.time.Clock()

    camera = Camera()
    SM.setMainCamera(camera)
    floor = Plane((0, -2, 10), 20, 20, plane=Physics.PLANE_XZ)
    r1 = Rigidbody.Rigidbody(floor)
    r1.mass=math.inf
    Rigidbody.addRigidbody(r1)
    floor.setRigidbody(r1)
    cube3 = Cube(position=(0.1, -5, 10.1), sideLength=1)
    r2 = Rigidbody.Rigidbody(cube3)
    Rigidbody.addRigidbody(r2)
    cube3.setRigidbody(r2)
    cube = Cube(position=(0, 0, 10), sideLength=1)
    cube2 = Cube(position=(0, 2, 10), sideLength=1)
    objectList = [cube, cube2, floor]
    controls = Controls.ControlManager(camera)
    rig = Rigidbody.Rigidbody(cube)
    Rigidbody.addRigidbody(rig)
    cube.setRigidbody(rig)
    rig2 = Rigidbody.Rigidbody(cube2)
    Rigidbody.addRigidbody(rig2)
    cube2.setRigidbody(rig2)
    collisions = []
    isPaused = False

    font = pygame.font.SysFont("Arial", 15)    
    pygame.mouse.set_pos((DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2))
    pygame.mouse.set_visible(False)
    pygame.event.set_grab(True)
    deltaTime = Clock.tick(FPS) / 1000

    
    cube.rigidbody.acceleration = Vector3(0, -2, 0)
    cube2.rigidbody.acceleration = Vector3(0, -2, 0)

    while True:
        pygame.display.update()
        deltaTime = Clock.tick(FPS) / 1000
        
        #update/render screen
        screen.fill(WHITE)
        SM.updateScreen(screen, objectList)
        #draw crosshair
        pygame.draw.line(screen, (255, 0, 0), (DISPLAY_WIDTH/2-5, DISPLAY_HEIGHT/2), (DISPLAY_WIDTH/2+5, DISPLAY_HEIGHT/2))
        pygame.draw.line(screen, (255, 0, 0), (DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2-5), (DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2+5))

        #manage user controls
        controls.checkUserInputs(deltaTime)
        
        #listen for pause requests
        if controls.checkPauseToggleRequest():
            isPaused = not isPaused
            controls.setPausedState(isPaused)
            pygame.mouse.set_visible(isPaused)
            pygame.mouse.set_pos((DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2))
            pygame.event.set_grab(not isPaused)

        #Debug
        position = "Pos: "
        for i in camera.position:
            position+= str(i) + " "
        cubeT = ""
        for i in cube.position:
            cubeT+= str(i) + " "
        cube2T = ""
        for i in cube2.position:
            cube2T+= str(i) + " "
        rot = "Rot: "
        for i in camera.rotation:
            rot+= str(i) + " "
        text1 = font.render(position, False, RED)
        rect1 = text1.get_rect()
        rect1.left = 100
        rect1.top = 0
        screen.blit(text1, rect1)
        text2 = font.render(cubeT, False, RED)
        rect2 = text2.get_rect()
        rect2.left = 100
        rect2.top = 20
        screen.blit(text2, rect2)
        text3 = font.render(cube2T, False, RED)
        rect3 = text3.get_rect()
        rect3.left = 100
        rect3.top = 40
        screen.blit(text3, rect3)
        text4 = font.render(rot, False, RED)
        rect4 = text4.get_rect()
        rect4.left = 100
        rect4.top = 60
        screen.blit(text4, rect4)
            
        #darken screen while paused, pause gameplay
        if isPaused:
            r = pygame.Rect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT)
            shape_surf = pygame.Surface(pygame.Rect(r).size, pygame.SRCALPHA)
            pygame.draw.rect(shape_surf, (0, 0, 0, 40), shape_surf.get_rect())
            screen.blit(shape_surf, r)
            continue

        #update physics
        if cube2.rigidbody.position[0] > 2.1:
            cube2.rigidbody.velocity = Vector3(-1, 0, 0)
        if cube.rigidbody.position[0] < -1.1:
            cube.rigidbody.velocity = Vector3(1, 0, 0)
        collisions = Rigidbody.checkAllCollisions()
        if collisions != None:
            Rigidbody.enactCollisions(collisions)
        PhysicsManager.update(deltaTime)


if __name__ == "__main__":
    main()
