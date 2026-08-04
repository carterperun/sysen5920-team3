from XRPLib.defaults import *
import time

base_effort = 0.25
KP = 0.6

while True:
    left = reflectance.get_left()
    right = reflectance.get_right()

    error = right - left

    print(f"L={left:.3f} R={right:.3f} error={error:+.3f}")
    drivetrain.set_effort(
        base_effort - error * KP,
        base_effort + error * KP
    )
    time.sleep(0.01)

# Run function here
line_track()
