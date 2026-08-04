# Import necessary modules
from machine import Pin, ADC
import bluetooth
import time
import math

from XRPLib.defaults import *
from pestolink import PestoLinkAgent

#Choose the name your robot shows up as in the Bluetooth paring menu
#Name should be 8 characters max!
robot_name = "T3amThr3"

# Create an instance of the PestoLinkAgent class
pestolink = PestoLinkAgent(robot_name)

def deadzone(value, threshold=0.1):
    if abs(value) < threshold:
        return 0.0
    return value

# Start an infinite loop
while True:
    if pestolink.is_connected():  # Check if a BLE connection is established
        rotation = deadzone(-1 * pestolink.get_axis(0))
        throttle = deadzone(-1 * pestolink.get_axis(1))
        
        drivetrain.arcade(throttle, rotation)
        
        if(pestolink.get_button(0)):
            servo_one.set_angle(110)
        else:
            servo_one.set_angle(90)
        
        batteryVoltage = (ADC(Pin("BOARD_VIN_MEASURE")).read_u16())/(1024*64/14)
        pestolink.telemetryPrintBatteryVoltage(batteryVoltage)

    else: #default behavior when no BLE connection is open
        drivetrain.arcade(0, 0)
        servo_one.set_angle(70)