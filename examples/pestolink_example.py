# Import necessary modules
from machine import Pin, ADC
import time
import math

from XRPLib.defaults import *
from pestolink import PestoLinkAgent

# robot name, bluetooth broadcasts this
robot_name = "H3ALTHY"

# Create an instance of the PestoLinkAgent class
pestolink = PestoLinkAgent(robot_name)


# drive 30 cm straight using library
def testStraightDrive():
    drivetrain.straight(30)
    return True

while True:
    if pestolink.is_connected():  
        rotation = -1 * pestolink.get_axis(0)
        throttle = -1 * pestolink.get_axis(1)

        if(pestolink.get_button(1)):
            testStraightDrive()

        drivetrain.arcade(throttle, rotation)
        
        if(pestolink.get_button(0)):
            servo_one.set_angle(110)
        else:
            servo_one.set_angle(90)
        
        batteryVoltage = (ADC(Pin("BOARD_VIN_MEASURE")).read_u16())/(1024*64/14)
        pestolink.telemetryPrintBatteryVoltage(batteryVoltage)

    else: 
        drivetrain.arcade(0, 0)

        servo_one.set_angle(70)


def performChallengeMaze():
    return True

def performChallengeTransporter():
    return True

def performChallengePatternMatch():
    return True

def performChallengeStacker():
    return True

def performChallengeMiniGolf():
    return True

def performChallengeSumo():
    return True

def performChallengeTowerOfHanoi():
    return True

def performChallengeBasketGallery():
    return True

def performChallengeTowerSmash():
    return True

def performChallengeLineFollower():
    return True

def performChallengeBallSearch():
    return True
