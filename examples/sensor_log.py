"""
sensor_log.py — SYSEN 5920 Team 3
Diagnostic logger: prints a full readout of every sensor and motor to the
console every 2 seconds. Use it to sanity-check wiring, calibrate
thresholds (line polarity, wall distances), and watch IMU drift live.

Run it from the XRP IDE with the robot connected over USB or the web IDE —
the output appears in the console/REPL pane. Move the robot by hand, wave
something in front of the rangefinder, slide it over the line tape, and
watch the numbers respond.

Press the USER button on the board to stop logging cleanly.

Readout guide:
  batt      battery voltage (motors need ~>7.0 V to behave)
  pwr       Y if the motor power switch is on and batteries connected
  yaw/pitch/roll   IMU orientation in degrees (yaw drifts slowly — watch
                   how fast; that is your maze-turn error budget)
  acc       accelerometer, milli-g. Flat and still should read ~0/0/+1000
  gyro      rotation rates, deg/s. Still robot should read ~0/0/0
  range     ultrasonic distance, cm (65535 = no echo/timeout)
  line L/R  IR reflectance 0..1 (low = light surface, high = dark). Put
            the robot on and off the tape and note both numbers — that
            is how you pick LINE_THRESHOLD / polarity for sumo_auto.py
  enc L/R   drivetrain encoder positions in cm since boot, and live RPM
  M3/M4     spare encoded-motor ports (position in revs, RPM)
  (servos have no position readback — nothing to log)
"""

from XRPLib.defaults import *
from machine import Pin, ADC
import time

LOG_PERIOD_S = 2.0

def battery_voltage():
    return ADC(Pin("BOARD_VIN_MEASURE")).read_u16() / (1024 * 64 / 14)

def main():
    board.led_blink(1)               # slow blink = logger running
    print("=" * 60)
    print("XRP sensor/motor logger - one block every 2 s")
    print("Press the USER button to stop.")
    print("=" * 60)

    t0 = time.ticks_ms()
    while not board.is_button_pressed():
        t = time.ticks_diff(time.ticks_ms(), t0) / 1000

        acc = imu.get_acc_rates()            # [x, y, z] milli-g
        gyro = imu.get_gyro_rates()          # [x, y, z] milli-deg/s

        print("-" * 60)
        print("t=%7.1fs | batt=%.2fV  pwr=%s  " % (
            t, battery_voltage(),
            "Y" if board.are_motors_powered() else "N"))
        print("  IMU   yaw=%+8.2f  pitch=%+7.2f  roll=%+7.2f  temp=%.1fC" % (
            imu.get_yaw(), imu.get_pitch(), imu.get_roll(),
            imu.temperature()))
        print("  ACC   x=%+6.0f y=%+6.0f z=%+6.0f mg" % (
            acc[0], acc[1], acc[2]))
        print("  GYRO  x=%+7.1f y=%+7.1f z=%+7.1f dps" % (
            gyro[0] / 1000, gyro[1] / 1000, gyro[2] / 1000))
        print("  RANGE %.1f cm" % rangefinder.distance())
        print("  LINE  L=%.3f  R=%.3f" % (
            reflectance.get_left(), reflectance.get_right()))
        print("  ENC   L=%+9.2fcm (%+6.1f RPM)   R=%+9.2fcm (%+6.1f RPM)" % (
            drivetrain.get_left_encoder_position(), left_motor.get_speed(),
            drivetrain.get_right_encoder_position(), right_motor.get_speed()))
        print("  M3    %+7.2frev (%+6.1f RPM)   M4  %+7.2frev (%+6.1f RPM)" % (
            motor_three.get_position(), motor_three.get_speed(),
            motor_four.get_position(), motor_four.get_speed()))

        # sleep in short slices so the USER button stays responsive
        slept = 0.0
        while slept < LOG_PERIOD_S:
            if board.is_button_pressed():
                break
            time.sleep(0.05)
            slept += 0.05

    board.led_off()
    print("=" * 60)
    print("Logger stopped by USER button.")

main()
