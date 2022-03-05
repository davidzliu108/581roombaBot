#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from turn import turnInPlace
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact, getCircumference, getTimeToDestinationInMS
from helperFunctions import waitForCenterButton


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

"""
class State(enum.Enum):    
    start = 0 # drive forward until wall
    startStop = 1 # after contact with wall, back up and turn 90deg (?) to right
    forward = 2 # drive foward until complete distance, bump wall, or distance exceeds threshold (20cm?)
    forwardBump = 3 # back up slightly? turn right
    forwardDistance = 4 # too far from wall, stop, turn left
    end = 5 # stop, done

state = State.start
"""
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

def startStop():
    moveForDistance(-1 * speed, 50, True)
    turnInPlace(speed/2, 90)
    return 2

def forwardBump():
    moveForDistance(-1 * speed, 50, True)
    turnInPlace(speed/2, 30)


state = 0
while state != 5:
    if state == 0: #david   
        # start
        start(speed)
        nextState = 1
    elif state == 1: #cody
        # startStop
        wait(500)
        nextState = startStop()
        nextState = 3
    elif state == 2: #david
        # forward
        print("do something")
        #forward(speed, 2000)
    elif state == 3: #cody
        # forwardBump
        waitForCenterButton()
        forwardBump()
    elif state == 4: #david
        # forwardDistance
        print("do something")
    elif state == 5: #cody
        # end
        print("do something")
    state = nextState


