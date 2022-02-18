from cmath import pi
from datetime import timedelta
from os import times
from threading import Timer
from time import time, time_ns
from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import *
#from constants import LARGE_WHEEL_DIAMETER_MM

def move(distanceInMM, speed): 
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.B)
    distanceTraveled = 0
    tireCircumfrence = getCircumference()
    time = (time_ns * 100000000)
    while (distanceTraveled < distanceInMM):
        leftMotor.run(speed)
        leftMotor.run(speed)
        currTime = (time_ns * 100000000)
        deltaTime = time - currTime
        distanceTraveled += getCircumference * speed * deltaTime



def getCircumference():
    LARGE_WHEEL_DIAMETER_MM = 68.8
    print(pi * LARGE_WHEEL_DIAMETER_MM)
    
