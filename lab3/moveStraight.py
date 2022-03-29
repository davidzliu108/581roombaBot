from operator import le
from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import *
import math
from constants import LARGE_WHEEL_DIAMETER_IN_MM

def stop():
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    leftMotor.hold()
    rightMotor.hold()

def moveForDistance(speed, distanceInMM, wait, distanceFromWallDelta): 
    distanceFromWallDelta = min(distanceFromWallDelta, 200)
    correction = 1
    direction = Direction.CLOCKWISE
    if (speed < 0):
        speed *= -1
        direction = Direction.COUNTERCLOCKWISE

    ev3 = EV3Brick()
    leftMotor = Motor(Port.A, direction)
    rightMotor = Motor(Port.D,  direction)

    timeNeeded = getTimeToDestinationInMS(distanceInMM, speed)
    leftMotor.run_time(speed + distanceFromWallDelta * correction, timeNeeded, Stop.HOLD, False)
    rightMotor.run_time(speed - distanceFromWallDelta * correction, timeNeeded, Stop.HOLD, wait)
    return

def moveUntilObstacle(speed, distToStopShort):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    sonar = UltrasonicSensor(Port.S2)
    
    leftMotor.run(speed)
    rightMotor.run(speed)
    distance = sonar.distance(False)
    while (distance > distToStopShort):
        distance = sonar.distance(False)
    leftMotor.hold()
    rightMotor.hold()
    return

def moveUntilContact(speed):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    leftMotor.run(speed)
    rightMotor.run(speed)
    paused = True
    while (paused):
        if touchSensorFront.pressed() == True:
            paused = False
            break
    
    leftMotor.hold()
    rightMotor.hold()
    return



def getCircumference():
    return LARGE_WHEEL_DIAMETER_IN_MM * math.pi

def getTimeToDestinationInMS(distance, speed):
    revolutions = distance / getCircumference()
    return math.trunc((revolutions * 360 / (speed)) * 1000)

def getDistanceTraveled(speed, time):
    degreesTraveled = speed * time / 1000
    revolutions = degreesTraveled / 360
    return getCircumference() * revolutions

    
