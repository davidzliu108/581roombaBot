#!/usr/bin/env pybricks-micropython
from pickle import TRUE
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from moveStraight import moveForDistance, moveUntilObstacle, moveUntilContact

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
ev3.speaker.beep()

speed = 10
moveForDistance(speed, 10) # (distance in MM, speed deg/sec )
ev3.speaker.beep()

while (TRUE): # pauses until button pressed
    pressed = ev3.buttons.pressed()
    for button in pressed:
        if (button == button.CENTER):
            break

#moveUntilObstacle(speed, 500) # not implemented yet
ev3.speaker.beep()

while (TRUE): # pauses until button pressed
    pressed = ev3.buttons.pressed()
    for button in pressed:
        if (button == button.CENTER):
            break

moveUntilContact(speed / 2)
moveForDistance(-speed, 10) # (distance in MM, speed deg/sec )
