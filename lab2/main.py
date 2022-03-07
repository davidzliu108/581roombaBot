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
    leftMotor.run(speed)
    rightMotor.run(speed)
    notReached = True
    resetAndStartWatch()
    sonar = UltrasonicSensor(Port.S2)
    innerBound = 60
    outerBound = 150
    while (notReached):
        wait(100)
        distanceRemaining -= getDistanceTraveled(speed, getDeltaTime())
        if distanceRemaining <= 0:
            notReached = False
            return 6
        if touchSensorFront.pressed() == True or touchSensorCorner.pressed() == True:
            notReached = True
            stop()
            return 3
        if (sonar.distance() >= outerBound): # too far away
            stop()
            return 4
        if (sonar.distance() < innerBound): # too close
            stop()
            return 5
    return 6

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

def forwardProximity():
    global distanceRemaining
    stop()
    sonar = UltrasonicSensor(Port.S2)
    innerBounds = 60
    resetAndStartWatch()
    while (sonar.distance() < innerBounds and distanceRemaining > 0):
        turnInPlace(speed, 15)
        moveForDistance(speed, 80, True)
        distanceRemaining -= getDistanceTraveled(speed, getTimeToDestinationInMS(80, speed))
    return

def forwardDistance():
    touchSensorFront = TouchSensor(Port.S1)
    touchSensorCorner = TouchSensor(Port.S3)
    global distanceRemaining
    stop()
    sonar = UltrasonicSensor(Port.S2)
    outerbounds = 150
    resetAndStartWatch()
    print("Moving left!!")
    distance = sonar.distance()
    prevDistance = 0
    while (distance > outerbounds and distanceRemaining > 0):
        print("Prev: " + str(prevDistance))
        print("Dist: " + str(distance))
        if distance > prevDistance:
            turnInPlace(speed, -25)
        if distance > 2000:
            return 2
        if (touchSensorFront.pressed() == True or touchSensorCorner.pressed() == True):
            stop()
            return 3
        moveForDistance(speed, 80, True)
        distanceRemaining -= getDistanceTraveled(speed, getTimeToDestinationInMS(80, speed))
        prevDistance = distance
        distance = sonar.distance()
    return 2
    '''
    ev3 = EV3Brick()
    sonar = UltrasonicSensor(Port.S2)

    global distanceRemaining
    resetAndStartWatch()
    print("Turning left")
    while (sonar.distance() > outerBound or distanceRemaining <= 0):
        print(sonar.distance())
        turnInPlace(speed, -25)
        moveForDistance(speed, 50, True)       
        distanceRemaining -= getDistanceTraveled(speed, getTimeToDestinationInMS(80, speed))
        if (sonar.distance(False) <= outerBound):
            stop()
            return 2
    stop()
    return 2
    '''


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
        nextState = forwardDistance()
    elif state == 5: #cody
        # forwardProximity == too close to wall
        forwardProximity()
        nextState = 2
    elif state == 6: #cody
        # end
        stop()
        ev3.speaker.beep()
        print("Finished!")
    state = nextState


