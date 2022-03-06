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

def forward(speed, distanceInMM):
    global distanceRemaining
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    touchSensorCorner = TouchSensor(Port.S3)

    #timeNeeded = getTimeToDestinationInMS(distanceRemaining, speed)
    print(distanceRemaining)
    leftMotor.run(speed)
    rightMotor.run(speed)
    notReached = True
    resetAndStartWatch()
    sonar = UltrasonicSensor(Port.S2)
    outerBound = 200
    while (notReached):
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
        if distanceRemaining <= 0:
            notReached = False
            return 6
        if touchSensorFront.pressed() == True or touchSensorCorner.pressed() == True:
            notReached = False
            stop()
            return 3
        if (sonar.distance(False) > outerBound): # too far away
            # do something
            stop()
            return 4
        if (): # too close
            # do something
    return

def forwardDistance(bound):
    ev3 = EV3Brick()
    sonar = UltrasonicSensor(Port.S2)

    global distanceRemaining
    distance = sonar.distance(False)
    turnInPlace(speed, -35)
    resetAndStartWatch()
    moveForDistance(speed, distanceRemaining, False)
    while (sonar.distance(False) > bound or distanceRemaining <= 0) :       
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
    
    distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
    stop()
    return

def startStop():
    moveForDistance(-1 * speed, 50, True)
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    turnInPlace(speed, 90)
    return

def forwardBump():
    moveForDistance(-1 * speed, 50, True)
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    wait(200)
    turnInPlace(speed, 35)

state = 0
speed = 200
watch = StopWatch()
distanceRemaining = 2000

sonar = UltrasonicSensor(Port.S2)

while state != 6:
    if state == 0: #david   
        # start
        start(speed)
        nextState = 1
    elif state == 1: #cody
        # startStop
        wait(200)
        startStop()
        nextState = 2
    elif state == 2: #david
        # forward
        nextState = forward(speed, distanceRemaining)
    elif state == 3: #cody
        # forwardBump
        forwardBump()
        nextState = 2
    elif state == 4: #david
        # forwardDistance
        forwardDistance(200)
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


