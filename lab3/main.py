#!/usr/bin/env pybricks-micropython
from os import urandom
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import pi, radians
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

def getAngle():
    global angle
    angle += gyro.angle()
    gyro.reset_angle(0)

def resetWatch():
    watch.reset()
    watch.resume()

def getDeltaTime():
    time = watch.time()
    watch.reset()
    return time

def start(speed):
    global traceStartPos, currPos
    gyro.reset_angle(0)
    resetWatch()
    moveUntilContact(speed)
    getAngle()
    traceStartPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed))
    currPos = traceStartPos
    print("Bump " + str(currPos))
    print("Gyro: " + str(gyro.angle()))
    return

def startStop():
    forwardBump()
    return

def forward(speed):
    resetWatch()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    global distanceRemaining, currPos
    global angle, gyroWatch, gyro
    if distanceRemaining <= 0:
        return 6
    touchSensorFront = TouchSensor(Port.S1)
    notReached = True
    sonar = UltrasonicSensor(Port.S2)
    idealDistance = 100
    started = False
    distanceFromWallDelta = 0
    while (notReached):
        # if (currPos[2] > pi):
        #     newheading = currPos[2] - 2 * pi
        #     currPos = (currPos[0], currPos[1], newheading)
        # elif (currPos[2] < 2 * pi):
        #     newheading = currPos[2] + 2 * pi
        #     currPos = (currPos[0], currPos[1], newheading)
        if started:
            currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(leftMotor.speed() + distanceFromWallDelta), radians(rightMotor.speed() - distanceFromWallDelta))
            print(currPos)
        else:
            resetWatch()
            started = True
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
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
        if distanceRemaining <= 0:
            notReached = False
            return 6
        if touchSensorFront.pressed() == True:
            print("Bumped")
            notReached = False
            stop()
            return 3
        wait(50)
    return 6

def forwardBump():
    global currPos, speed
    resetWatch()
    moveForDistance(-1 * speed, 60, True, 0)
    currPos =  calculatePosition(currPos, getDeltaTime() / 1000, radians(speed * -1), radians(speed * -1))
    print("After contact: " + str(currPos))
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    resetWatch()
    turnInPlace(speed, 90)
    currPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed * -1))
    print("After Turn: " + str(currPos))
    print(str(gyro.angle()))


    


state = 0
speed = 200
distanceRemaining = 1000

sonar = UltrasonicSensor(Port.S2)
gyro = GyroSensor(Port.S3, Direction.COUNTERCLOCKWISE)
gyro.reset_angle(0)
gyroWatch = StopWatch()
gyroWatch.reset()
gyroWatch.resume()
watch = StopWatch()
angle = 0
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
    elif state == 1: #cody
        # startStop
        startStop()
        nextState = 6
    elif state == 2: #david
        # forward
        nextState = forward(speed)
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
