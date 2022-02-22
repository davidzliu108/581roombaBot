from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import *
import math
#from constants import LARGE_WHEEL_DIAMETER_MM

def moveForDistance(speed, distanceInMM): 
    ev3 = EV3Brick()
    print("test")
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)

    timeNeeded = getTimeToDestinationInMS(distanceInMM, speed)
    leftMotor.run_time(speed, timeNeeded, Stop.BRAKE, False)
    rightMotor.run_time(speed, timeNeeded, Stop.BRAKE, False)
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
    while (True):
        if touchSensor.pressed:
            break
    return



def getCircumference():
    LARGE_WHEEL_DIAMETER_MM = 56
    return LARGE_WHEEL_DIAMETER_MM * math.pi

def getTimeToDestinationInMS(distance, speed):
    revolutions = distance / getCircumference()
    return math.trunc(revolutions * 360 / (speed)) * 1000

    
