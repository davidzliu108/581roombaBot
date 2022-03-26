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
    r = calculateR(leftSpeed, rightSpeed)
    return (currPosition[0] - r * sin(currPosition[2]), currPosition[1] + r * cos(currPosition[2]))

def calculateTheta(heading, deltaTime, leftSpeed, rightSpeed):
    return heading + calculateW(leftSpeed, rightSpeed) * deltaTime

def calculateW(leftSpeed, rightSpeed):
    w = (rightSpeed*radius - leftSpeed*radius)/robotWidth
    return w

def calculatePosition(currPosition, deltaTime, leftSpeed, rightSpeed):
    icc = calculateICC(currPosition, leftSpeed, rightSpeed)
    print(icc)
    thetaPrime = calculateTheta(currPosition[2], deltaTime, leftSpeed, rightSpeed)
    print(thetaPrime)
    if(leftSpeed == rightSpeed):
        posPrime = calculatePositionWhenStraight(currPosition, leftSpeed, deltaTime)
        print(posPrime)
        return posPrime
    xPrime = (currPosition[0] - icc[0]) * cos(thetaPrime) - (currPosition[1] - icc[1]) * sin(thetaPrime) + icc[0]
    yPrime = (currPosition[0] - icc[0]) * sin(thetaPrime) + (currPosition[1] - icc[1]) * cos(thetaPrime) + icc[1]
    print(xPrime)
    print(yPrime)
    return (xPrime, yPrime, thetaPrime)

def calculatePositionWhenStraight(currPosition, speed, deltaTime):
    print("New Pos:")
    print(speed)
    print(deltaTime)
    xPrime = currPosition[0] + speed * radius * cos(currPosition[2]) * deltaTime
    yPrime = currPosition[1] + speed * radius * sin(currPosition[2]) * deltaTime
    return (xPrime, yPrime, currPosition[2])

def comparePosition(targetPosition, currPosition):
    maxDistance = 100
    distance = math.sqrt((targetPosition[0] - currPosition[0])**2 + (targetPosition[1] - currPosition[1])**2)
    if (distance < maxDistance):
        return True
    return False
