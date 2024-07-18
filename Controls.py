#Controls.py

import sys
import pygame

from Constants import *
import ScreenManager

class ControlManager:
    def __init__(self, camera):
        self.camera = camera
        self.togglePause = False
        self.isPaused = False
        self.firstCall = True

    #manage moving camera controls
    def checkUserInputs(self, deltaTime):
        #camera position movement
        if pygame.key.get_pressed()[pygame.K_a]:
            self.camera.moveRelHorz((-MOVE_SPEED * deltaTime, 0, 0))
        if pygame.key.get_pressed()[pygame.K_d]:
            self.camera.moveRelHorz((MOVE_SPEED * deltaTime, 0, 0))
        if pygame.key.get_pressed()[pygame.K_w]:
            self.camera.moveRelHorz((0, 0, MOVE_SPEED * deltaTime))
        if pygame.key.get_pressed()[pygame.K_s]:
            self.camera.moveRelHorz((0, 0, -MOVE_SPEED * deltaTime))
        if pygame.key.get_pressed()[pygame.K_SPACE]:
            self.camera.moveRelHorz((0, MOVE_SPEED * deltaTime, 0))
        if pygame.key.get_pressed()[pygame.K_LCTRL]:
            self.camera.moveRelHorz((0, -MOVE_SPEED * deltaTime, 0))

        for event in pygame.event.get():
            #camera turning movement
            if event.type == pygame.MOUSEMOTION and not self.isPaused and self.firstCall != True:
                if event.pos != (DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2):
                    pos = [event.pos[0]-DISPLAY_WIDTH/2, event.pos[1]-DISPLAY_HEIGHT/2]
                    self.camera.rotate((TURN_SPEED * pos[0] * deltaTime,- TURN_SPEED * pos[1] * deltaTime))
                    pygame.mouse.set_pos((DISPLAY_WIDTH/2, DISPLAY_HEIGHT/2))
            #pausing
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.togglePause = True
            #quit event handle
            elif event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            #negate first call jank
            if self.firstCall:
                self.firstCall = False


    #called by main thread, checks for pause request
    def checkPauseToggleRequest(self):
        if self.togglePause:
            self.togglePause = False
            return True
        return False

    def setPausedState(self, state):
        self.isPaused = state



        
