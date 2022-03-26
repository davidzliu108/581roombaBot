from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color


robotWidth = 136.525
radius = 28

def waitForCenterButton():
    ev3 = EV3Brick()
    while (True): # pauses until button pressed
        pressed = ev3.buttons.pressed()
        for button in pressed:
            if (button == button.CENTER):
                return

def calculateR():
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    r = ((rightMotor.speed()*radius + leftMotor.speed()*radius)/(rightMotor.speed()*radius - leftMotor.speed()*radius)) * (robotWidth/2)
    return r

def calculateW():
    ev3 = EV3Brick()
    leftMotor = Motor(Port.A)
    rightMotor = Motor(Port.D)
    w = (rightMotor.speed()*radius - leftMotor.speed()*radius)/robotWidth
    return w