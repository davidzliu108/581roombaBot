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
    direction = Direction.CLOCKWISE
    if (speed < 0):
        speed *= -1
        direction = Direction.COUNTERCLOCKWISE

    ev3 = EV3Brick()
    leftMotor = Motor(Port.A, direction)
    rightMotor = Motor(Port.D,  direction)

    timeNeeded = getTimeToDestinationInMS(distanceInMM, speed)
    """
    print(timeNeeded)
    leftMotor.run(speed)
    rightMotor.run(speed)
    wait(timeNeeded)
    print("stopping")
    leftMotor.hold
    rightMotor.hold
    """
    leftMotor.run_time(speed, timeNeeded, Stop.COAST, False)
    rightMotor.run_time(speed, timeNeeded, Stop.COAST, True)
    return

def moveUntilObstacle(speed, distToStopShort):
    print("test Move sensor")
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    sonar = UltrasonicSensor(Port.S2)
    
    leftMotor.run(speed)
    rightMotor.run(speed)
    distance = sonar.distance(False)
    while (distance > distToStopShort):
        distance = sonar.distance(False)
        #print(distance)
    
    print("Stopping")
    leftMotor.hold()
    rightMotor.hold()
    return

def moveUntilContact(speed):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensor = TouchSensor(Port.S1)

    leftMotor.run(speed)
    rightMotor.run(speed)
    paused = True
    while (paused):
        if touchSensor.pressed() == True:
            paused = False
            break
    
    leftMotor.hold()
    rightMotor.hold()
    wait(1000)
    return



def getCircumference():
    LARGE_WHEEL_DIAMETER_MM = 56
    return LARGE_WHEEL_DIAMETER_MM * math.pi

def getTimeToDestinationInMS(distance, speed):
    revolutions = distance / getCircumference()
    return math.trunc(revolutions * 360 / (speed)) * 1000

    
