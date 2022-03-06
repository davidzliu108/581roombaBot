#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from turn import turnInPlace, leftCorrect
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact, getCircumference, getTimeToDestinationInMS, stop, getDistanceTraveled
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

distanceRemaining = 2000
speed = 200
watch = StopWatch()


def resetAndStartWatch():
    watch.reset()
    watch.resume()

def getDeltaTime():
    time = watch.time()
    watch.reset()
    return time

def start(speed):
    moveUntilContact(speed)
    return

def forward(speed, distanceInMM):
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    touchSensorCorner = TouchSensor(Port.S3)

    timeNeeded = getTimeToDestinationInMS(distanceInMM, speed)

    leftMotor.run_time(speed, timeNeeded, Stop.COAST, False)
    rightMotor.run_time(speed, timeNeeded, Stop.COAST, False)
    notReached = True
    resetAndStartWatch()
    while (notReached):
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
        if touchSensorFront.pressed() == True or touchSensorCorner.pressed() == True:
            notReached = False
            leftMotor.hold()
            rightMotor.hold()
            return
    return

def forwardDistance(bound):
    ev3 = EV3Brick()
    sonar = UltrasonicSensor(Port.S2)

    distance = sonar.distance(False)

    while (distance > bound) :
        moveForDistance(-1 * speed, 50, True)
        leftCorrect(speed, 90)
        return
    return

def startStop():
    moveForDistance(-1 * speed, 50, True)
    distanceRemaining -= getDistanceTraveled(speed, getTimeToDestinationInMS(50, speed))
    turnInPlace(speed, 90)
    return 2

def forwardBump():
    moveForDistance(-1 * speed, 50, True)
    distanceRemaining -= getDistanceTraveled(speed, getTimeToDestinationInMS(50, speed))
    wait(500)
    turnInPlace(speed, 35)

state = 0
while state != 6:
    if state == 0: #david   
        # start
        start(speed)
        nextState = 1
    elif state == 1: #cody
        # startStop
        wait(200)
        nextState = startStop()
        nextState = 2
    elif state == 2: #david
        # forward
        forward(speed, 2000)
        nextState = 3
    elif state == 3: #cody
        # forwardBump
        forwardBump()
        nextState = 2
    elif state == 4: #david
        # forwardDistance
        forwardDistance(290)
        nextState = 2
    elif state == 5: #cody
        # forwardProximity == too close to wall
        stop()
        turnInPlace(speed, 15)
        nextState = 2
    elif state == 6: #cody
        # end
        stop()
        print("Finished!")
    state = nextState


