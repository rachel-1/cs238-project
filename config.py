DRONE_MAX_ACCEL = 1
DRONE_MAX_SPEED = 1

ACCEL_STEP = 0.25
SPEED_STEP = 0.25
DIST_STEP = 0.25

DIST_VARIANCE = .1
SPEED_VARIANCE = .1

ACCEL_PENALTY = 4
SPEED_PENALTY = 3

BUS_SPEED = 2

MAX_LOCAL_STEPS = 100

DISCOUNT = 0.98

INVALID_ACTION_PENALTY = 5

# if this is not true, things get messed up
assert ACCEL_STEP == SPEED_STEP == DIST_STEP
