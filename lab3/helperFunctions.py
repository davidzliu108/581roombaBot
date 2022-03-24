from pybricks.hubs import EV3Brick

def waitForCenterButton():
    ev3 = EV3Brick()
    while (True): # pauses until button pressed
        pressed = ev3.buttons.pressed()
        for button in pressed:
            if (button == button.CENTER):
                return