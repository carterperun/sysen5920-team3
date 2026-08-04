from XRPLib.defaults import *
import time

def line_track():
    # Set base effort value
    base_effort = 0.3
    # Set KP value
    KP = 0.6
    # Put in while loop so it runs infinitely
    while True:
        # Calculate error (fixed hyphen to dot)
        error = reflectance.get_left() - reflectance.get_right()
        # Tell drivetrain to set effort values on right and left motors based on error
        drivetrain.set_effort(base_effort - error * KP, base_effort + error * KP)
        # Re-calculate values every 0.01 seconds
        time.sleep(0.01)

# Run function here
line_track()
