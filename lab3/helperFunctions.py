from tokenize import Triple
from typing import Tuple
from pybricks.hubs import EV3Brick
from math import sin, cos

def waitForCenterButton():
    ev3 = EV3Brick()
    while (True): # pauses until button pressed
        pressed = ev3.buttons.pressed()
        for button in pressed:
            if (button == button.CENTER):
                return

def calculateR():
    print("")

def calculateICC(currPosition, heading):
    return Tuple(currPosition.x - calculateR() * sin(heading), currPosition.y + calculateR * cos(heading))

def calculateHeading(heading, deltaTime):
    return heading + calculateW() * deltaTime
