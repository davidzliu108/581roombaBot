#!/usr/bin/env pybricks-micropython
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

def calculateNeededTurn():
    global angle, currPost, goalPos, gyro
    desiredHeading = 0.0
    desiredHeading = getAngleToFacePoint(currPos, goalPos)
    desiredHeading = (desiredHeading + 180) % 360 - 180
    currentHeading = (angle + 180) % 360 - 180
    neededTurnDeg = desiredHeading - currentHeading   #  degrees(currentHeading) - desiredHeading
    neededTurnDeg = (neededTurnDeg + 180) % 360 - 180
    return trunc(neededTurnDeg)

def getAngle():
    global angle, currPos, gyro
    angle += gyro.angle()
    gyro.reset_angle(0)
    return angle
    
    
def findDistance(a, b):
    squaredXs = (b[0] - a[0]) ** 2
    squaredYs = (b[1] - a[1]) ** 2
    return (squaredXs + squaredYs) ** 0.5

def checkIfAtDestination(destination):
    global currPos, leftTraceStart, counter
    tolerance = 100 # in mm
    distance = findDistance(currPos, destination)
    counter = counter + 1
    if (counter >= 10):
        counter = 0
        # print("Angle:" + str(angle))
        print("Pos: " + str(currPos))
        print("Distance: " + str(distance))
    return distance <= tolerance


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
    mCount = 0 # quick and dirty count to wait a set amount of time before checking if we are at the m line
    while (notReached):
        if started:
            getAngle()
            currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(leftMotor.speed() + distanceFromWallDelta), radians(rightMotor.speed() - distanceFromWallDelta), radians(angle))
        else:
            resetWatch()
            started = True
        if mCount >= 50:
            done = checkIfAtDestination(goalPos)
            distanceFromMLine = distanceMLine(startPos, goalPos)
            #print("Distance from M: " + str(distanceFromMLine))
            ACCEPTANCE_RADIUS = 100
            if (distanceFromMLine <= 100):
                stop()
                return 1
            if (done): 
                return 6
        else:
            mCount += 1
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
    ERROR_CORRECTION = 1
    print("Moving towards goal")
    getAngle()
    print("Current Heading: " + str(angle))
    desiredHeading = 0.0
    desiredHeading = getAngleToFacePoint(currPos, goalPos)
    desiredHeading = (desiredHeading + 180) % 360 - 180
    currentHeading = (angle + 180) % 360 - 180
    neededTurnDeg = desiredHeading - currentHeading   #  degrees(currentHeading) - desiredHeading
    neededTurnDeg = (neededTurnDeg + 180) % 360 - 180
    neededTurnDeg = trunc(neededTurnDeg * ERROR_CORRECTION)
    print("Turning: " + str(-neededTurnDeg))
    resetWatch()
    turnInPlace(speed, -neededTurnDeg)

    if (neededTurnDeg > 0): 
            currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(speed), radians(speed * -1), radians(getAngle()))
    elif (neededTurnDeg < 0):
            currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(speed * -1), radians(speed), radians(getAngle()))
    # while abs(neededTurnDeg) > 5.0:
    #     resetWatch()
    #     turnInPlace(speed, -neededTurnDeg)

    #     if (neededTurnDeg > 0): 
    #             currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(speed), radians(speed * -1), radians(getAngle()))
    #     elif (neededTurnDeg < 0):
    #             currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(speed * -1), radians(speed), radians(getAngle()))
    #     else:
    #         break
    #     desiredHeading = 0.0
    #     desiredHeading = getAngleToFacePoint(currPos, (1800, 1800))
    #     desiredHeading = (desiredHeading + 180) % 360 - 180
    #     currentHeading = (angle + 180) % 360 - 180
    #     neededTurnDeg = desiredHeading - currentHeading    #  degrees(currentHeading) - desiredHeading
    #     neededTurnDeg = (neededTurnDeg + 180) % 360 - 180
    #     neededTurnDeg = trunc(neededTurnDeg * ERROR_CORRECTION)
    #     print("needed: " + str(neededTurnDeg))
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    leftMotor.run(speed)
    rightMotor.run(speed)
    leftSpeedDiff = 0
    rightSpeedDiff = 0
    dCount = 0
    while (True):
        wait(50)
        dCount += 1
        currPos = calculatePosition(currPos, getDeltaTime()/1000, radians(speed + leftSpeedDiff), radians(speed + rightSpeedDiff), radians(getAngle()))
        desiredHeading = 0.0
        desiredHeading = getAngleToFacePoint(currPos, goalPos)
        desiredHeading = (desiredHeading + 180) % 360 - 180
        currentHeading = (angle + 180) % 360 - 180
        neededTurnDeg = desiredHeading - currentHeading
        neededTurnDeg = (neededTurnDeg + 180) % 360 - 180
        neededTurnDeg = trunc(neededTurnDeg * ERROR_CORRECTION * 1)
        speedDiff = 0
        if (abs(neededTurnDeg) != 0):
            if (neededTurnDeg < 0):
                speedDiff = max(neededTurnDeg * 10, -100)
                leftSpeedDiff = speedDiff  *-1
                rightSpeedDiff = speedDiff
            else:
                speedDiff = min(neededTurnDeg * 10, 100)
                leftSpeedDiff = speedDiff *-1
                rightSpeedDiff = speedDiff
        else:
            leftSpeedDiff = 0
            rightSpeedDiff = 0
        if dCount >= 10:
            print("Angle: " + str(currentHeading))
            print("Desired heading: " + str(desiredHeading))
            print("Needed turn: " + str(neededTurnDeg))
        finished = checkIfAtDestination(goalPos)
        if (finished):
            stop()
            print("finished")
            return 4
        if (touchSensorFront.pressed()):
            stop()
            return 2
        leftMotor.run(speed + leftSpeedDiff)
        rightMotor.run(speed + rightSpeedDiff) 
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
gyro.reset_angle(90)
gyroWatch = StopWatch()
gyroWatch.reset()
gyroWatch.resume()
watch = StopWatch()
angle = 0
distanceFromDest = 0

startPos = (1000, 0.0, 1.5708)
goalPos = (1800, 1800, 0.0)
traceStartPos = (0.0, 0.0, 1.5708)
currPos = (1000, 0.0, 1.5708)
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
