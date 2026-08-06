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

PIVOT_EFFORT = 0.65
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


def pivot_turn(pestolink, degrees):
    """Pivot to an IMU target. Positive is left; negative is right."""
    target = normalize_angle(imu.get_yaw() + degrees)
    start = time.ticks_ms()

    print(
        "Turning {} 90 degrees: yaw={:.1f}, target={:.1f}".format(
            "left" if degrees > 0 else "right",
            imu.get_yaw(),
            target,
        )
    )

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

            if error > 0:
                # Left: drive the right wheel and rest the left wheel.
                drivetrain.set_effort(0, PIVOT_EFFORT)
            else:
                # Right: drive the left wheel and rest the right wheel.
                drivetrain.set_effort(PIVOT_EFFORT, 0)

            time.sleep(0.01)
    finally:
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
                pivot_turn(pestolink, 90)

            elif pestolink.get_button(DPAD_RIGHT):
                wait_for_release(pestolink, DPAD_RIGHT)
                if pestolink.get_button(STOP_BUTTON):
                    break
                pivot_turn(pestolink, -90)

            time.sleep(0.02)
    finally:
        drivetrain.stop()
        board.led_off()


main()
