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

def startStop():
    moveForDistance(-1 * speed, 60, True, 0)
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    turnInPlace(speed, 90)
    return

def forward(speed, distanceInMM):
    global distanceRemaining
    if distanceRemaining <= 0:
        return 6
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    notReached = True
    resetAndStartWatch()
    sonar = UltrasonicSensor(Port.S2)
    idealDistance = 100
    while (notReached):
        wait(100)
        distanceFromWall = sonar.distance()
        distanceFromWallDelta = idealDistance - distanceFromWall
        negative = distanceFromWallDelta < 0
        distanceFromWallDelta = min(abs(distanceFromWallDelta), 80)
        if (negative):
            distanceFromWallDelta = distanceFromWallDelta * -1
        else:
            distanceFromWallDelta * 3
        print(distanceFromWallDelta)
        leftMotor.run(speed + distanceFromWallDelta)
        rightMotor.run(speed - distanceFromWallDelta)
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
        if distanceRemaining <= 0:
            notReached = False
            return 6
        if touchSensorFront.pressed() == True:
            notReached = False
            stop()
            return 3
    return

def forwardBump():
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    moveForDistance(-1 * speed, 50, True, 0)
    wait(200)
    turnInPlace(speed, 90)
    

state = 0
speed = 200
watch = StopWatch()
distanceRemaining = 2000

sonar = UltrasonicSensor(Port.S2)

inProgress = True
while inProgress:
    if state == 0: #david   
        # start
        waitForCenterButton()
        start(speed)
        nextState = 1
    elif state == 1: #cody
        # startStop
        wait(100)
        startStop()
        nextState = 2
    elif state == 2: #david
        # forward
        nextState = forward(speed, distanceRemaining)
    elif state == 3: #cody
        # forwardBump
        forwardBump()
        nextState = 2
    elif state == 6: #cody
        # end
        stop()
        inProgress = False
        ev3.speaker.beep()
        print("Finished!")
        
    state = nextState
