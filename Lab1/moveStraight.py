from cmath import pi
from curses import BUTTON1_CLICKED
from os import truncate
from pickle import FALSE, TRUE
from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import *
#from constants import LARGE_WHEEL_DIAMETER_MM

def moveForDistance(distanceInMM, speed): 
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.B)

    timeNeeded = getTimeToDestination(distanceInMM, speed)
    leftMotor.run_time(speed, timeNeeded, Stop, TRUE)
    rightMotor.run_time(speed, timeNeeded, Stop, TRUE)
    return

def moveUntilObstacle(speed, distToStopShort):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.B)
    sonar = UltrasonicSensor(Port.C)

    distance = sonar.distance
    while (sonar.distance > distToStopShort):
        distance = sonar.distance
    
    leftMotor.hold
    rightMotor.hold
    return

def moveUntilContact(speed):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.B)
    touchSensor = TouchSensor(Port.C)

    leftMotor.run(speed)
    rightMotor.run(speed)
    while (TRUE):
        if touchSensor.pressed:
            break
    return



def getCircumference():
    LARGE_WHEEL_DIAMETER_MM = 56

def getTimeToDestination(distance, speed):
    revolutions = distance / getCircumference()
    return truncate(revolutions * 360 / speed)

    
