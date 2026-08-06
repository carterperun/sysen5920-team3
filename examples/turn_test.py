"""Test 90-degree pivot turns with a PestoLink controller.

D-pad LEFT  (button 14): pivot 90 degrees left
D-pad RIGHT (button 15): pivot 90 degrees right
START       (button 9):  stop motors and end the test
"""

from XRPLib.defaults import *
from pestolink import PestoLinkAgent
import time


ROBOT_NAME = "T3amThr3"

DPAD_LEFT = 14
DPAD_RIGHT = 15
STOP_BUTTON = 9

TURN_EFFORT_HIGH = 0.80
TURN_EFFORT_LOW = 0.55
SLOW_ZONE_DEG = 20.0
TURN_TOLERANCE_DEG = 3.0
TURN_TIMEOUT_S = 5.0
SETTLE_TIME_S = 0.25


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def stop_requested(pestolink):
    return (
        not pestolink.is_connected()
        or pestolink.get_button(STOP_BUTTON)
    )


def in_place_turn(pestolink, degrees):
    """Counter-rotate both wheels to an IMU target."""
    target = normalize_angle(imu.get_yaw() + degrees)
    start = time.ticks_ms()

    print(
        "Turning {} 90 degrees: yaw={:.1f}, target={:.1f}".format(
            "left" if degrees > 0 else "right",
            imu.get_yaw(),
            target,
        )
    )

    drivetrain.set_zero_effort_behavior(True)
    try:
        while True:
            if stop_requested(pestolink):
                print("Turn aborted")
                return False

            error = normalize_angle(target - imu.get_yaw())
            if abs(error) <= TURN_TOLERANCE_DEG:
                print("Turn complete: yaw={:.1f}".format(imu.get_yaw()))
                return True

            if time.ticks_diff(time.ticks_ms(), start) > \
                    TURN_TIMEOUT_S * 1000:
                print(
                    "Turn timed out: yaw={:.1f}, error={:.1f}".format(
                        imu.get_yaw(), error
                    )
                )
                return False

            effort = TURN_EFFORT_HIGH
            if abs(error) <= SLOW_ZONE_DEG:
                effort = TURN_EFFORT_LOW

            if error > 0:
                # Left: left wheel backward, right wheel forward.
                drivetrain.set_effort(-effort, effort)
            else:
                # Right: left wheel forward, right wheel backward.
                drivetrain.set_effort(effort, -effort)

            time.sleep(0.01)
    finally:
        drivetrain.stop()
        time.sleep(0.10)  # brake briefly instead of coasting past the target
        drivetrain.set_zero_effort_behavior(False)
        drivetrain.stop()
        time.sleep(SETTLE_TIME_S)


def wait_for_release(pestolink, button):
    while pestolink.is_connected() and pestolink.get_button(button):
        if pestolink.get_button(STOP_BUTTON):
            break
        time.sleep(0.02)


def main():
    pestolink = PestoLinkAgent(ROBOT_NAME)
    drivetrain.stop()
    board.led_on()

    print("Waiting for controller...")
    print("D-pad LEFT/RIGHT tests turns; START ends the test.")

    try:
        while True:
            if not pestolink.is_connected():
                drivetrain.stop()
                time.sleep(0.05)
                continue

            if pestolink.get_button(STOP_BUTTON):
                print("START pressed; ending turn test")
                break

            if pestolink.get_button(DPAD_LEFT):
                wait_for_release(pestolink, DPAD_LEFT)
                if pestolink.get_button(STOP_BUTTON):
                    break
                in_place_turn(pestolink, 90)

            elif pestolink.get_button(DPAD_RIGHT):
                wait_for_release(pestolink, DPAD_RIGHT)
                if pestolink.get_button(STOP_BUTTON):
                    break
                in_place_turn(pestolink, -90)

            time.sleep(0.02)
    finally:
        drivetrain.stop()
        board.led_off()


main()
