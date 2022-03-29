from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import *
import math
from moveStraight import getTimeToDestinationInMS
from constants import LARGE_WHEEL_DIAMETER_IN_MM
from moveStraight import getCircumference

robotWidth = robotWidth = 140


def getTimeToTurnInMSFromDeg(deg, speed):
    revolution = deg / 360
    distance = robotWidth * math.pi * revolution
    revolutions = distance / getCircumference()
    time = math.trunc((revolutions * 360 / (speed)) * 1000)
    return time


def turnInPlace(speed, deg):
    if (deg < 0):
        leftMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        rightMotor = Motor(Port.D,  Direction.CLOCKWISE)
        deg = deg * -1
    else:
        leftMotor = Motor(Port.A, Direction.CLOCKWISE)
        rightMotor = Motor(Port.D,  Direction.COUNTERCLOCKWISE)
    
    timeNeeded = getTimeToTurnInMSFromDeg(deg, speed)
    leftMotor.run_time(speed, timeNeeded, Stop.HOLD, False)
    rightMotor.run_time(speed, timeNeeded, Stop.HOLD, True)
    return

def leftCorrect(speed, deg):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    rightMotor = Motor(Port.D,  Direction.CLOCKWISE)
    timeNeeded = getTimeToTurnInMSFromDeg(deg, speed)
    leftMotor.run_time(speed, timeNeeded, Stop.HOLD, False)
    rightMotor.run_time(speed, timeNeeded, Stop.HOLD, True)
    return