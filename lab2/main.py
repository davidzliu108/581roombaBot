#!/usr/bin/env pybricks-micropython
from unittest import case
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from enum import Enum
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact, getCircumference, getTimeToDestinationInMS
from helperFunctions import waitForCenterButton


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

class State(Enum):
    start = 0 # drive forward until wall
    startStop = 1 # after contact with wall, back up and turn 90deg (?) to right
    forward = 2 # drive foward until complete distance, bump wall, or distance exceeds threshold (20cm?)
    forwardBump = 3 # back up slightly? turn right
    forwardDistance = 4 # too far from wall, stop, turn left
    end = 5 # stop, done

state = State.start

speed = 300




def start(speed):
    moveUntilContact(speed/2)
    return

def forward(speed, distanceInMM):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensor = TouchSensor(Port.S1)

    timeNeeded = getTimeToDestinationInMS(distanceInMM, speed)

    leftMotor.run_time(speed, timeNeeded, Stop.COAST, False)
    rightMotor.run_time(speed, timeNeeded, Stop.COAST, True)

    notReached = True
    while (notReached):
        if touchSensor.pressed() == True:
            notReached = False
            leftMotor.hold()
            rightMotor.hold()
            return
    return

#def forwardDistance()



while state != State.end:
    if state == State.start: #david
        print("do something")
        moveUntilContact(speed/2)
    elif state == State.startStop: #cody
        print("do something")
    elif state == State.forward: #david
        print("do something")
        forward(speed, 2000)
    elif state == State.forwardBump: #cody
        print("do something")
    elif state == State.forwardDistance: #david
        print("do something")
    elif state == State.end: #cody
        print("do something") 


