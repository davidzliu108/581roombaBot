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

def getTimeToTurnInMSFromDeg(deg, speed):
    revolution = deg / 360
    botDiameterInMM = 133.35
    distance = botDiameterInMM * math.pi * revolution
    print("Distance: " + str(distance))
    revolutions = distance / getCircumference()
    time = math.trunc((revolutions * 360 / (speed)) * 1000)
    print("Time: " + str(time))
    return time


def turnInPlace(speed, deg):
    ev3 = EV3Brick()
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