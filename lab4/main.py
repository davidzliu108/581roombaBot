# Cody Irion PID: 702442575
# David Liu PID: 730317472
# 
#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import pi, radians, degrees,trunc,sqrt, cos, sin
from helperFunctions import calculatePosition
from turn import turnInPlace, leftCorrect
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact, getCircumference, getTimeToDestinationInMS, stop, getDistanceTraveled
from helperFunctions import waitForCenterButton, getAngleToFacePoint
import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# L = 136.525mm
# radius of wheel = 28mm

# Create your objects here.
ev3 = EV3Brick()

def calculateNeededTurn():
    global angle, currPost, goalPos, gyro, counter
    desiredHeading = 0.0
    desiredHeading = getAngleToFacePoint(currPos, goalPos)
    desiredHeading = (desiredHeading + 180) % 360 - 180
    currentHeading = (angle + 180) % 360 - 180
    neededTurnDeg = desiredHeading - currentHeading   #  degrees(currentHeading) - desiredHeading
    neededTurnDeg = (neededTurnDeg + 180) % 360 - 180
    if (counter >= 20):
        print
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
    tolerance = 150 # in mm
    distance = findDistance(currPos, destination)
    counter = counter + 1
    if (counter >= 10):
        counter = 0
        print("Pos: " + str(currPos))
        print("Distance: " + str(distance))
    return distance <= tolerance

def getDestinationMP(destination, distAdd):
    global angle
    return (destination[0]+math.cos(getAngle())*distAdd, destination[1]+math.sin(getAngle())*distAdd)


def resetWatch():
    watch.reset()
    watch.resume()

def getDeltaTime():
    time = watch.time()
    watch.reset()
    return time / 1000

def traceObstacle():
    resetWatch()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    global currPos, traceStartPos, leftTraceStart
    global angle, gyroWatch, gyro, speed
    touchSensorFront = TouchSensor(Port.S1)
    touchSensorFrontR = TouchSensor(Port.S3)
    notReached = True
    sonar = UltrasonicSensor(Port.S2)
    idealDistance = 100
    started = False
    distanceFromWallDelta = 0
    mCount = 0 # quick and dirty count to wait a set amount of time before checking if we are at the m line
    while (notReached):
        if started:
            currPos = calculatePosition(currPos, getDeltaTime(), radians(leftMotor.speed() + distanceFromWallDelta), radians(rightMotor.speed() - distanceFromWallDelta), radians(getAngle()))
        else:
            resetWatch()
            started = True
        if mCount >= 50:
            
            distanceFromMLine = distanceMLine(startPos, goalPos)
            if (distanceFromMLine <= 100):
                stop()
                return 1
        else:
            mCount += 1
        done = checkIfAtDestination(goalPos)
        if (done): 
                return 4
        distanceFromWall = sonar.distance()
        distanceFromWallDelta = idealDistance - distanceFromWall
        negative = distanceFromWallDelta < 0
        distanceFromWallDelta = min(abs(distanceFromWallDelta), 120)
        if (negative):
            distanceFromWallDelta = distanceFromWallDelta * -1
        else:
            distanceFromWallDelta * 3
        leftMotor.run(speed + distanceFromWallDelta)
        rightMotor.run(speed - distanceFromWallDelta)
        if touchSensorFront.pressed() or touchSensorFrontR.pressed():
            notReached = False
            stop()
            return 3
        wait(50)
    return 2

def forwardBump():
    global currPos, speed
    resetWatch()
    moveForDistance(-1 * speed, 120, True, 0)
    getAngle()
    currPos =  calculatePosition(currPos, getDeltaTime(), radians(speed * -1), radians(speed * -1), radians(angle))
    resetWatch()
    turnInPlace(speed / 2, 90)
    getAngle()
    currPos = calculatePosition(currPos, getDeltaTime(), radians(speed), radians(speed * -1), radians(angle))

def driveTowardsGoal():
    global currPos, goalPos
    ACCEPTANCE_RADIUS = 100
    ERROR_CORRECTION = 1
    getAngle()
    print("Current Heading: " + str(angle))
    neededTurnDeg = calculateNeededTurn()
    print("Turning: " + str(-neededTurnDeg))
    resetWatch()
    turnInPlace(speed, -neededTurnDeg)

    if (neededTurnDeg > 0): 
            currPos = calculatePosition(currPos, getDeltaTime(), radians(speed), radians(speed * -1), radians(getAngle()))
    elif (neededTurnDeg < 0):
            currPos = calculatePosition(currPos, getDeltaTime(), radians(speed * -1), radians(speed), radians(getAngle()))
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    touchSensorFront = TouchSensor(Port.S1)
    touchSensorFrontR = TouchSensor(Port.S3)
    leftMotor.run(speed)
    rightMotor.run(speed)
    leftSpeedDiff = 0
    rightSpeedDiff = 0
    while (True):
        wait(50)
        currPos = calculatePosition(currPos, getDeltaTime(), radians(speed + leftSpeedDiff), radians(speed + rightSpeedDiff), radians(getAngle()))
        neededTurnDeg = calculateNeededTurn()
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
        finished = checkIfAtDestination(goalPos)
        if (finished):
            stop()
            print("finished")
            return 4
        if (touchSensorFront.pressed() or touchSensorFrontR.pressed()):
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
speed = 350
counter = 19 ## prints every 20th cycle starting with the first
sonar = UltrasonicSensor(Port.S2)
gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)
gyro.reset_angle(90)
gyroWatch = StopWatch()
gyroWatch.reset()
gyroWatch.resume()
watch = StopWatch()
angle = 0
startPos = (1000, 0.0, 1.5708)
goalPos = (1800, 1800, 0.0)
currPos = (1000, 0.0, 1.5708)
inProgress = True
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
        wait(200)
        ev3.speaker.play_file(SoundFile.T_REX_ROAR)
        print("Finished!")
        
    state = nextState
