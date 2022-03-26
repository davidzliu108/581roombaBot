from tokenize import Triple
from typing import Tuple
from pybricks.hubs import EV3Brick
from math import sin, cos
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
import math


robotWidth = 136.525
radius = 28

def waitForCenterButton():
    ev3 = EV3Brick()
    while (True): # pauses until button pressed
        pressed = ev3.buttons.pressed()
        for button in pressed:
            if (button == button.CENTER):
                return

def calculateR():
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    r = ((rightMotor.speed()*radius + leftMotor.speed()*radius)/(rightMotor.speed()*radius - leftMotor.speed()*radius)) * (robotWidth/2)
    return r

def calculateICC(currPosition, heading):
    return Tuple(currPosition.x - calculateR() * sin(heading), currPosition.y + calculateR * cos(heading))

def calculateHeading(heading, deltaTime):
    return heading + calculateW() * deltaTime

def calculateW():
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    w = (rightMotor.speed()*radius - leftMotor.speed()*radius)/robotWidth
    return w


def comparePosition(targetPosition, currPosition):
    maxDistance = 100
    distance = math.sqrt((targetPosition[0] - currPosition[0])**2 + (targetPosition[1] - currPosition[1])**2)
    if (distance < maxDistance):
        return True