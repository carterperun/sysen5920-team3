from machine import Pin, ADC
import bluetooth
import time
import math

from XRPLib.defaults import *
from pestolink import PestoLinkAgent

robot_name = "T3amThr3"

pestolink = PestoLinkAgent(robot_name)

drivetrain.set_zero_effort_behavior(EncodedMotor.ZERO_EFFORT_BREAK)

def deadzone(value, threshold=0.1):
    if abs(value) < threshold:
        return 0.0
    return value

FWD_HEADING_KP = 0.03
FWD_HEADING_KD = 0.004
FWD_MAX_CORRECTION = 0.2
REV_HEADING_KP = 0.04
REV_MAX_CORRECTION = 0.4
heading_target = None
prev_yaw = 0.0
prev_yaw_ms = time.ticks_ms()

def performChallenge1():
    return True

def performChallenge2():
    return True

def performChallenge3():
    return True

def performChallenge4():
    return True

def performChallenge5():
    return True

def performChallenge6():
    return True

def performChallenge7():
    return True

def performChallenge8():
    return True

def performChallenge9():
    return True

def performChallenge10():
    return True

def performChallenge11():
    return True

while True:
    if pestolink.is_connected():
        rotation = deadzone(-1 * pestolink.get_axis(0))
        throttle = deadzone(-1 * pestolink.get_axis(1))

        yaw_now = imu.get_yaw()
        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, prev_yaw_ms) / 1000
        yaw_rate = (yaw_now - prev_yaw) / dt if dt > 0 else 0.0
        prev_yaw = yaw_now
        prev_yaw_ms = now_ms

        if throttle != 0 and rotation == 0:
            if heading_target is None:
                heading_target = yaw_now
            if throttle > 0:
                error = yaw_now - heading_target
                correction = FWD_HEADING_KP * error + FWD_HEADING_KD * yaw_rate
                correction = max(-FWD_MAX_CORRECTION, min(FWD_MAX_CORRECTION, correction))
            else:
                error = heading_target - yaw_now
                correction = REV_HEADING_KP * error
                correction = max(-REV_MAX_CORRECTION, min(REV_MAX_CORRECTION, correction))
            drivetrain.arcade(throttle, correction)
        else:
            heading_target = None
            drivetrain.arcade(throttle, rotation)

        if(pestolink.get_button(0)):
            servo_one.set_angle(110)
        else:
            servo_one.set_angle(90)

        batteryVoltage = (ADC(Pin("BOARD_VIN_MEASURE")).read_u16())/(1024*64/14)
        pestolink.telemetryPrintBatteryVoltage(batteryVoltage)

    else:
        heading_target = None
        drivetrain.arcade(0, 0)
        servo_one.set_angle(70)
