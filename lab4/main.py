# Cody Irion PID: 702442575
#  David Liu PID: 730317472
# 
# !/usr/bin/env pybricks-micropython
from turtle import distance
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import pi, radians, degrees,trunc,sqrt
from helperFunctions import calculatePosition
from turn import turnInPlace, leftCorrect
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact, getCircumference, getTimeToDestinationInMS, stop, getDistanceTraveled
from helperFunctions import waitForCenterButton, getAngleToFacePoint
import math

###### NEEDED METHODS #######
# Methods: findDistanceFromLine, driveTowardsGoal
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
    return angle
    
    
def findDistance(a, b):
    b = getDestinationMP(b, 150)
    squaredXs = (b[0] - a[0]) ** 2
    squaredYs = (b[1] - a[1]) ** 2
    return (squaredXs + squaredYs) ** 0.5

def checkIfAtDestination(destination):
    global currPos, leftTraceStart, counter
    tolerance = 200 # in mm
    distance = findDistance(currPos, destination)
    counter = counter + 1
    if (counter >= 10):
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


def getDestinationMP(destination, distAdd):
    global angle
    return (destination[0]+math.cos(getAngle())*distAdd, destination[1]+math.sin(getAngle())*distAdd)


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
    global currPos, speed, traceStartPos, distanceToWallFromStart
    resetWatch()
    moveForDistance(-1 * speed, 130, True, 0)
    getAngle()
    currPos =  calculatePosition(currPos, getDeltaTime() / 1000, radians(speed * -1), radians(speed * -1), radians(angle))
    distanceToWallFromStart = findDistance((0,0), currPos)
    traceStartPos = currPos

    print("Trace: " + str(traceStartPos))
    ev3.speaker.beep()
    resetWatch()
    turnInPlace(speed, 90)
    getAngle()
    currPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed * -1), radians(angle))
    return

def traceObstacle():
    resetWatch()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    global currPos, traceStartPos, leftTraceStart
    global angle, gyroWatch, gyro, speed
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
            done = checkIfAtDestination(goalPos)
            distanceFromMLine = distanceFromMLine(startPos, goalPos)
            ACCEPTANCE_RADIUS = 100
            if (distanceFromMLine <= 100):
                stop()
                return 1
            if (done): 
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
        if touchSensorFront.pressed() == True:
            notReached = False
            stop()
            return 3
        wait(50)
    return 2

def forwardBump():
    global currPos, speed
    resetWatch()
    moveForDistance(-1 * speed, 90, True, 0)
    getAngle()
    currPos =  calculatePosition(currPos, getDeltaTime() / 1000, radians(speed * -1), radians(speed * -1), radians(angle))
    resetWatch()
    turnInPlace(speed, 90)
    getAngle()
    currPos = calculatePosition(currPos, getDeltaTime() / 1000, radians(speed), radians(speed * -1), radians(angle))

def driveTowardsGoal():
    global currPos, goalPos
    ACCEPTANCE_RADIUS = 100
    ERROR_CORRECTION = .8
    print("Moving towards goal")
    neededTurnDeg = 0.0
    desiredHeading = 0.0
    desiredHeading = getAngleToFacePoint(currPos, (1800, 1800))
    desiredHeading = (desiredHeading + 360) % 360
    currentHeading = (degrees(currPos[2]) + 360) % 360
    neededTurnDeg = desiredHeading - currentHeading    #  degrees(currentHeading) - desiredHeading
    neededTurnDeg = (neededTurnDeg + 180) % 360 - 180
    neededTurnDeg = trunc(neededTurnDeg * ERROR_CORRECTION)
    resetWatch()
    turnInPlace(speed, -neededTurnDeg)
    if (neededTurnDeg > 0): 
            currPos = calculatePosition(currPos, getDeltaTime(), speed, speed * -1, radians(getAngle()))
    elif (neededTurnDeg < 0):
            currPos = calculatePosition(currPos, getDeltaTime(), speed * -1, speed, radians(getAngle()))
    
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    leftMotor.run(speed)
    rightMotor.run(speed)
    while (True):
        wait(100)
        currPos = calculatePosition(currPos, getDeltaTime(), speed, speed, radians(getAngle()))
        distanceFromGoal = distance(currPos, goalPos)
        if (distance < ACCEPTANCE_RADIUS):
            stop()
            return 3
        if (touchSensorFront.pressed()):
            stop()
            return 2
    return


def distanceMLine(start, end):
    numerator = abs((end[0] - start[0])*(start[1] - currPos[1]) - (start[0] - currPos[0])*(end[1] - start[1]))
    denominator = sqrt((end[0]-start[0])**2 + (end[1] - start[1])**2)
    return numerator / denominator
    

state = 0
speed = 200
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

startPos = (1000, 0.0, 0.0)
goalPos = (1800, 1800, 0.0)
traceStartPos = (0.0, 0.0, 0.0)
currPos = (1000, 0.0, 0.0)
inProgress = True
leftTraceStart = False
distanceToWallFromStart = 0.0

while inProgress:
    if state == 0: #david   
        # start
        waitForCenterButton()
        nextState = 1
    elif state == 1: #cody
        # move Towards Goal
        nextState = driveTowardsGoal()
    elif state == 2: #cody
        # trade Obstacle
        nextState = traceObstacle()
    elif state == 3:
        forwardBump()
        nextState = 2
    elif state == 4: #cody
        # end
        stop()
        inProgress = False
        ev3.speaker.beep()
        print("Finished!")
        
    state = nextState
