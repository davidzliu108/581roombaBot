#!/usr/bin/env pybricks-micropython
from turtle import st
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
from helperFunctions import waitForCenterButton, getAngleToFacePoint
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
    global angle, currPos, gyro
    angle += gyro.angle()
    gyro.reset_angle(0)
    
def findDistance(a, b):
    squaredXs = (b[0] - a[0]) ** 2
    squaredYs = (b[1] - a[1]) ** 2
    return (squaredXs + squaredYs) ** 0.5

def checkIfAtDestination(destination):
    global currPos, leftTraceStart, counter
    tolerance = 150 # in mm
    distance = findDistance(currPos, destination)
    counter = counter + 1
    if (counter == 20):
        counter = 0
        print("Angle:" + str(angle))
        print("Pos: " + str(currPos))
        print("Distance: " + str(distance))
    if (leftTraceStart):
        leftTraceStart = True
        done = distance <= tolerance
        return done
    else: 
        return distance > tolerance * 1.25


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
    print("Start: " + str(gyro.angle()))
    resetWatch()
    moveUntilContact(speed)
    getAngle()
    print("After Bump: " + str(angle)) 
    currPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed), radians(angle))
    return

def startStop():
    global currPos, speed, traceStartPos
    resetWatch()
    moveForDistance(-1 * speed, 60, True, 0)
    getAngle()
    currPos =  calculatePosition(currPos, getDeltaTime() / 1000, radians(speed * -1), radians(speed * -1), radians(angle))
    traceStartPos = currPos
    print("Trace: " + str(traceStartPos))
    ev3.speaker.beep()
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    resetWatch()
    turnInPlace(speed, 90)
    getAngle()
    currPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed * -1), radians(angle))
    return

def forward(speed):
    resetWatch()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    global distanceRemaining, currPos, traceStartPos, leftTraceStart
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
        if started:
            getAngle()
            currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(leftMotor.speed() + distanceFromWallDelta), radians(rightMotor.speed() - distanceFromWallDelta), radians(angle))
        else:
            resetWatch()
            started = True
        if leftTraceStart:
            print("Trace debug: " + str(traceStartPos))
            done = checkIfAtDestination(traceStartPos)
            if (done): 
                checkIfAtDestination(traceStartPos)
                return 6
        else:
            leftTraceStart = checkIfAtDestination(traceStartPos)
            if leftTraceStart: print("Left trace start radius")
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
        if distanceRemaining <= 0:
            notReached = False
            return 6
        if touchSensorFront.pressed() == True:
            notReached = False
            stop()
            return 3
        wait(50)
    return 6

def forwardBump():
    global currPos, speed
    resetWatch()
    moveForDistance(-1 * speed, 60, True, 0)
    getAngle()
    currPos =  calculatePosition(currPos, getDeltaTime() / 1000, radians(speed * -1), radians(speed * -1), radians(angle))
    global distanceRemaining
    distanceRemaining -= getDistanceTraveled(-1 * speed, getTimeToDestinationInMS(50, speed))
    resetWatch()
    turnInPlace(speed, 90)
    getAngle()
    currPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed * -1), radians(angle))

def returnToStart():
    desiredHeading = getAngleToFacePoint((74.25572001155581, 126.9482924725219, 4.433136300065597), (0, 0))
    neededTurnDeg = currPos[2] - desiredHeading
    resetWatch()
    turnInPlace(speed, neededTurnDeg)
    if (neededTurnDeg > 0): 
        currPos = calculatePosition(currPos, getDeltaTime(), speed, speed * -1, radians(angle))
    elif (neededTurnDeg < 0):
        currPos = calculatePosition(currPos, getDeltaTime(), speed * -1, speed, radians(angle))
    distance = findDistance(currPos, (0, 0))
    moveForDistance(speed, distance, True, 0)

    

state = 0
speed = 200
distanceRemaining = 100
counter = 19 ## prints every 20th cycle starting with the first
sonar = UltrasonicSensor(Port.S2)
gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)
gyro.reset_angle(0)
gyroWatch = StopWatch()
gyroWatch.reset()
gyroWatch.resume()
watch = StopWatch()
angle = 0
distanceFromDest = 0
startPos = (0.0, 0.0, 0.0)
traceStartPos = (0.0, 0.0, 0.0)
currPos = (0.0, 0.0, 0.0)
inProgress = True
leftTraceStart = False
while inProgress:
    if state == 0: #david   
        # start
        waitForCenterButton()
        start(speed)
        nextState = 1
    elif state == 1: #cody
        # startStop
        startStop()
        nextState = 2
    elif state == 2: #david
        # forward
        nextState = forward(speed)
    elif state == 3: #cody
        # forwardBump
        forwardBump()
        nextState = 2
    elif state == 4: 
        # return to start
        returnToStart()
    elif state == 6: #cody
        # end
        stop()
        inProgress = False
        ev3.speaker.beep()
        print("Finished!")
        
    state = nextState
