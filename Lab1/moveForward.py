from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from constants import LARGE_WHEEL_DIAMETER_MM

def move(distanceInCM, topSpeed): 
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.B)
    
