from pybricks.hubs import EV3Brick
from math import sin, cos
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
import math


#robotWidth = 160.3 # mid
#robotWidth = 190 # full
# 160 was close
robotWidth = 160
#robotWidth = (190 + 160.3) / 2
#robotWidth = 108 # inside
radius = 28.0

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

def calculatePosition(currPosition, deltaTime, leftSpeed, rightSpeed, heading):
    icc = calculateICC(currPosition, leftSpeed, rightSpeed)
    thetaPrime = heading #calculateTheta(currPosition[2], deltaTime, leftSpeed, rightSpeed)
    if(leftSpeed == rightSpeed):
        posPrime = calculatePositionWhenStraight(currPosition, leftSpeed, deltaTime, heading)
        return posPrime
    #print("curving")
    xPrime = ((currPosition[0] - icc[0]) * cos(calculateW(leftSpeed, rightSpeed) * deltaTime) - (currPosition[1] - icc[1]) * sin(calculateW(leftSpeed, rightSpeed) * deltaTime)) + icc[0]
    yPrime = ((currPosition[0] - icc[0]) * sin(calculateW(leftSpeed, rightSpeed) * deltaTime) + (currPosition[1] - icc[1]) * cos(calculateW(leftSpeed, rightSpeed) * deltaTime)) + icc[1]
    return (xPrime, yPrime, thetaPrime)

def calculatePositionWhenStraight(currPosition, speed, deltaTime, heading):
    #print("Straight")
    xPrime = currPosition[0] + speed * radius * deltaTime * cos(heading)
    yPrime = currPosition[1] + speed * radius  * deltaTime * sin(heading)
    return (xPrime, yPrime, currPosition[2])

def comparePosition(targetPosition, currPosition):
    maxDistance = 100
    distance = math.sqrt((targetPosition[0] - currPosition[0])**2 + (targetPosition[1] - currPosition[1])**2)
    if (distance < maxDistance):
        return True
    return False

def getAngleToFacePoint(start, end):
    angle= math.atan2(end[0] - start[0], end[1] - start[1])
    angle = angle * (180 / math.pi)
    if (angle < 0):
        angle = 360 - (-angle)
    return math.trunc(angle * -1)