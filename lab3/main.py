#!/usr/bin/env pybricks-micropython
import string
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import radians
from helperFunctions import calculatePosition
from turn import turnInPlace, leftCorrect
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact, getCircumference, getTimeToDestinationInMS, stop, getDistanceTraveled
from helperFunctions import waitForCenterButton

###### NEEDED METHODS #######
# Methods: calculateR(return float), calculateW(return float), calculateICC(return (x, y)), CalculateTheta(return float), calculateDistanceWhenStraight, calculateDistTurning,
#          comparePosition
#       
#



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# L = 136.525mm
# radius of wheel = 28mm


# Create your objects here.
ev3 = EV3Brick()

def resetAndStartWatch():
    watch.reset()
    watch.resume()

def getDeltaTime():
    time = watch.time()
    watch.reset()
    return time

def start(speed):
    deltaTime = 0
    watch.resume()
    moveUntilContact(speed)
    watch.pause()
    deltaTime = watch.time()
    global traceStartPos, currPos
    traceStartPos = calculatePosition(currPos, deltaTime / 1000, radians(speed), radians(speed))
    currPos = traceStartPos
    #print(traceStartPos)
    return

def startStop():
    global currPos, speed
    watch.resume()
    moveForDistance(-1 * speed, 60, True, 0)
    watch.pause()
    currPos =  calculatePosition(currPos, watch.time() / 1000, radians(speed * -1), radians(speed * -1))
    print("After contact: " + str(currPos))
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    watch.reset()
    watch.resume()
    turnInPlace(speed, 90)
    watch.pause()
    currPos = calculatePosition(currPos, watch.time() / 1000, radians(speed), radians(speed * -1))
    print("After Turn: " + str(currPos))
    return

def forward(speed, distanceInMM):
    time = 0
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    global distanceRemaining, currPos
    if distanceRemaining <= 0:
        return 6
    touchSensorFront = TouchSensor(Port.S1)
    notReached = True
    resetAndStartWatch()
    sonar = UltrasonicSensor(Port.S2)
    idealDistance = 100
    # distanceFromWall = sonar.distance()
    # distanceFromWallDelta = idealDistance - distanceFromWall
    # leftMotor.run(speed + distanceFromWallDelta)
    # rightMotor.run(speed - distanceFromWallDelta)
    # currPos = calculatePosition(currPos, 100/1000, radians(leftMotor.speed() + distanceFromWallDelta), radians(rightMotor.speed() - distanceFromWallDelta))
    #wait(100)
    while (notReached):
        time = watch.time()
        resetAndStartWatch()
        distanceFromWall = sonar.distance()
        distanceFromWallDelta = idealDistance - distanceFromWall
        negative = distanceFromWallDelta < 0
        distanceFromWallDelta = min(abs(distanceFromWallDelta), 80)
        if (negative):
            distanceFromWallDelta = distanceFromWallDelta * -1
        else:
            distanceFromWallDelta * 3
        leftMotor.run(speed + distanceFromWallDelta)
        rightMotor.run(speed - distanceFromWallDelta)
        print(time/1000)
        currPos = calculatePosition(currPos, time/1000, radians(leftMotor.speed() + distanceFromWallDelta), radians(rightMotor.speed() - distanceFromWallDelta))
        print(currPos)
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
        if distanceRemaining <= 0:
            notReached = False
            return 6
        if touchSensorFront.pressed() == True:
            print("Bumped")
            notReached = False
            stop()
            return 3
        #wait(100)
    return

def forwardBump():
    global currPos, speed
    watch.resume()
    moveForDistance(-1 * speed, 60, True, 0)
    watch.pause()
    currPos =  calculatePosition(currPos, watch.time() / 1000, radians(speed * -1), radians(speed * -1))
    print("After contact: " + str(currPos))
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    watch.reset()
    watch.resume()
    turnInPlace(speed, 90)
    currPos = calculatePosition(currPos, watch.time() / 1000, radians(speed), radians(speed * -1))
    print("After Turn: " + str(currPos))


    


state = 0
speed = 200
distanceRemaining = 1000

sonar = UltrasonicSensor(Port.S2)
watch = StopWatch()
startPos = (0.0, 0.0, 0.0)
traceStartPos = (0.0, 0.0, 0.0)
currPos = (0.0, 0.0, 0.0)
inProgress = True
while inProgress:
    if state == 0: #david   
        # start
        waitForCenterButton()
        start(speed)
        nextState = 1
        resetAndStartWatch()
    elif state == 1: #cody
        # startStop
        startStop()
        nextState = 2
        resetAndStartWatch()
    elif state == 2: #david
        # forward
        nextState = forward(speed, distanceRemaining)
        resetAndStartWatch()
    elif state == 3: #cody
        # forwardBump
        forwardBump()
        nextState = 2
        resetAndStartWatch()
    elif state == 6: #cody
        # end
        stop()
        inProgress = False
        ev3.speaker.beep()
        print("Finished!")
        
    state = nextState
