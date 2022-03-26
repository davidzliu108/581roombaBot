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

def calculateR(leftSpeed, rightSpeed):
    ev3 = EV3Brick()
    vr = leftSpeed*radius
    vl = rightSpeed*radius
    if (rightSpeed == leftSpeed):
        return 0
    r = ((vr + vl)/(vr - vl)) * (robotWidth/2)
    return r


def calculateICC(currPosition, leftSpeed, rightSpeed):
    print("calculating ICC")
    r = calculateR(leftSpeed, rightSpeed)
    return (currPosition[0] - r * sin(currPosition[2]), currPosition[1] + r * cos(currPosition[2]))

def calculateTheta(heading, deltaTime):
    return heading + calculateW() * deltaTime

def calculateW():
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    w = (rightMotor.speed()*radius - leftMotor.speed()*radius)/robotWidth
    return w

def calculatePosition(currPosition, deltaTime, leftSpeed, rightSpeed):
    icc = calculateICC(currPosition, leftSpeed, rightSpeed)
    thetaPrime = calculateTheta(currPosition[2], deltaTime)
    xPrime = (currPosition[0] - icc[0]) * cos(thetaPrime) - (currPosition[1] - icc[1]) * sin(thetaPrime) + icc[0]
    yPrime = (currPosition[0] - icc[0]) * sin(thetaPrime) + currPosition[1] - icc[1] * cos(thetaPrime) + icc[1]
    return (xPrime, yPrime, thetaPrime)

def comparePosition(targetPosition, currPosition):
    maxDistance = 100
    distance = math.sqrt((targetPosition[0] - currPosition[0])**2 + (targetPosition[1] - currPosition[1])**2)
    if (distance < maxDistance):
        return True
    return False
