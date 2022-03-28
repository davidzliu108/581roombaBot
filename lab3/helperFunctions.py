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
    vl = leftSpeed*radius
    vr = rightSpeed*radius
    if (rightSpeed == leftSpeed):
        return 0
    r = ((vl + vr)/(vr - vl)) * (robotWidth/2)
    return r


def calculateICC(currPosition, leftSpeed, rightSpeed):
    if (leftSpeed == rightSpeed):
        return currPosition
    r = calculateR(leftSpeed, rightSpeed)
    icc = (currPosition[0] - r * sin(currPosition[2]), currPosition[1] + r * cos(currPosition[2]))
    return icc

def calculateTheta(heading, deltaTime, leftSpeed, rightSpeed):
    thetaPrime = heading + calculateW(leftSpeed, rightSpeed) * deltaTime
    return thetaPrime

def calculateW(leftSpeed, rightSpeed):
    w = (rightSpeed*radius - leftSpeed*radius)/robotWidth
    return w

def calculatePosition(currPosition, deltaTime, leftSpeed, rightSpeed):
    print("curving")
    icc = calculateICC(currPosition, leftSpeed, rightSpeed)
    thetaPrime = calculateTheta(currPosition[2], deltaTime, leftSpeed, rightSpeed)
    if(leftSpeed == rightSpeed):
        posPrime = calculatePositionWhenStraight(currPosition, leftSpeed, deltaTime)
        return posPrime
    xPrime = ((currPosition[0] - icc[0]) * cos(calculateW(leftSpeed, rightSpeed) * deltaTime) - (currPosition[1] - icc[1]) * sin(calculateW(leftSpeed, rightSpeed) * deltaTime)) + icc[0]
    yPrime = ((currPosition[0] - icc[0]) * sin(calculateW(leftSpeed, rightSpeed) * deltaTime) + (currPosition[1] - icc[1]) * cos(calculateW(leftSpeed, rightSpeed) * deltaTime)) + icc[1]
    return (xPrime, yPrime, thetaPrime)

def calculatePositionWhenStraight(currPosition, speed, deltaTime):
    print("Straight")
    xPrime = currPosition[0] + speed * radius * cos(currPosition[2]) * deltaTime
    yPrime = currPosition[1] + speed * radius * sin(currPosition[2]) * deltaTime
    return (xPrime, yPrime, currPosition[2])

def comparePosition(targetPosition, currPosition):
    maxDistance = 100
    distance = math.sqrt((targetPosition[0] - currPosition[0])**2 + (targetPosition[1] - currPosition[1])**2)
    if (distance < maxDistance):
        return True
    return False