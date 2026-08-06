from XRPLib.defaults import *
from pestolink import PestoLinkAgent
import time


# Standalone controller configuration.
ROBOT_NAME = "T3amThr3"
START_BUTTON = 2  # Y on the standard PestoLink gamepad mapping

# Maze geometry. Coordinates are (column, row).
GRID_WIDTH = 4
GRID_HEIGHT = 6
START = (0, 0)
GOAL = (0, 5)

# The robot starts facing north. Headings increase clockwise:
# 0 = north, 1 = east, 2 = south, 3 = west.
START_HEADING = 0

# Motion and sensor tuning values.
CELL_DISTANCE_CM = 30.48
DRIVE_EFFORT = 0.45
TURN_EFFORT_HIGH = 0.80
TURN_EFFORT_LOW = 0.55
TURN_SLOW_ZONE_DEG = 20.0
TURN_TOLERANCE_DEG = 3.0
WALL_DISTANCE_CM = 20.0
SETTLE_TIME_S = 0.20
SENSOR_SAMPLES = 5
MOTION_TIMEOUT_S = 5.0

# Change these if testing shows that the robot travels or turns too far.
CELL_DISTANCE_SCALE = 1.0
TURN_ANGLE_SCALE = 1.0

DIRECTIONS = (
    (0, 1),    # north
    (1, 0),    # east
    (0, -1),   # south
    (-1, 0),   # west
)
DIRECTION_NAMES = ("north", "east", "south", "west")


def average_distance():
    """Return an averaged ultrasonic reading in centimeters."""
    readings = []
    for _ in range(SENSOR_SAMPLES):
        distance = rangefinder.distance()
        if distance > 0:
            readings.append(distance)
        time.sleep(0.03)

    if not readings:
        return 0.0
    return sum(readings) / len(readings)


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def in_place_turn(degrees):
    """Counter-rotate both wheels to an IMU target."""
    target = normalize_angle(imu.get_yaw() + degrees)
    start = time.ticks_ms()

    drivetrain.set_zero_effort_behavior(True)
    try:
        while True:
            error = normalize_angle(target - imu.get_yaw())
            if abs(error) <= TURN_TOLERANCE_DEG:
                return

            if time.ticks_diff(time.ticks_ms(), start) > \
                    MOTION_TIMEOUT_S * 1000:
                raise RuntimeError("In-place turn timed out")

            effort = TURN_EFFORT_HIGH
            if abs(error) <= TURN_SLOW_ZONE_DEG:
                effort = TURN_EFFORT_LOW

            if error > 0:
                drivetrain.set_effort(-effort, effort)
            else:
                drivetrain.set_effort(effort, -effort)

            time.sleep(0.01)
    finally:
        drivetrain.stop()
        time.sleep(0.10)
        drivetrain.set_zero_effort_behavior(False)
        drivetrain.stop()


def turn_right():
    in_place_turn(-90 * TURN_ANGLE_SCALE)
    time.sleep(SETTLE_TIME_S)


def turn_left():
    in_place_turn(90 * TURN_ANGLE_SCALE)
    time.sleep(SETTLE_TIME_S)


def adjacent_cell(position, heading):
    dx, dy = DIRECTIONS[heading]
    return position[0] + dx, position[1] + dy


def is_inside_maze(position):
    x, y = position
    return 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT


def path_is_open(position, heading):
    """Check both the known grid boundary and the physical wall sensor."""
    next_position = adjacent_cell(position, heading)
    if not is_inside_maze(next_position):
        print("Grid boundary detected")
        return False

    distance = average_distance()
    print("Ultrasonic distance: {:.1f} cm".format(distance))
    return distance > WALL_DISTANCE_CM


def move_one_cell(position, heading):
    if not drivetrain.straight(
        CELL_DISTANCE_CM * CELL_DISTANCE_SCALE,
        DRIVE_EFFORT,
        timeout=MOTION_TIMEOUT_S,
    ):
        raise RuntimeError("One-cell movement timed out")
    time.sleep(SETTLE_TIME_S)
    return adjacent_cell(position, heading)


def solve_maze():
    """Follow the right wall until GOAL is reached."""
    position = START
    heading = START_HEADING

    print("Starting at {} facing {}".format(
        position, DIRECTION_NAMES[heading]
    ))
    time.sleep(1)

    try:
        while position != GOAL:
            # Check the right-hand path.
            turn_right()
            heading = (heading + 1) % 4

            if path_is_open(position, heading):
                position = move_one_cell(position, heading)
            else:
                # Restore the original direction and check forward.
                turn_left()
                heading = (heading - 1) % 4

                if path_is_open(position, heading):
                    position = move_one_cell(position, heading)
                else:
                    # Check the left-hand path.
                    turn_left()
                    heading = (heading - 1) % 4

                    if path_is_open(position, heading):
                        position = move_one_cell(position, heading)
                    else:
                        # The robot is in a dead end. It is already facing
                        # left, so one more left turn faces backward.
                        turn_left()
                        heading = (heading - 1) % 4

                        if not path_is_open(position, heading):
                            raise RuntimeError(
                                "No open path found in any direction"
                            )
                        position = move_one_cell(position, heading)

            print("Now at {} facing {}".format(
                position, DIRECTION_NAMES[heading]
            ))

        drivetrain.stop()
        print("Goal reached at {}".format(position))

    except BaseException:
        drivetrain.stop()
        raise


def wait_for_start(pestolink):
    """Keep the robot parked until the controller's Y button is pressed."""
    drivetrain.stop()
    print("Waiting for PestoLink controller and Y button...")

    while True:
        if pestolink.is_connected() and pestolink.get_button(START_BUTTON):
            break
        time.sleep(0.02)

    # Wait for release so the held start press cannot trigger anything else.
    while pestolink.is_connected() and pestolink.get_button(START_BUTTON):
        time.sleep(0.02)


def main():
    pestolink = PestoLinkAgent(ROBOT_NAME)
    wait_for_start(pestolink)
    print("Y pressed; starting maze solver")
    solve_maze()


main()
