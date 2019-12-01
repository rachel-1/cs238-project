# 6 m/s^2; DJI drone could accelerate to 50 mph in 4 sec
DRONE_MAX_ACCEL = 1#6 

# 25 m/s ~= 56 mph; DJI around 50 mph, top racing drones max out at 100 mph
DRONE_MAX_SPEED = 1#25 

ACCEL_STEP = 0.5
SPEED_STEP = 0.5
DIST_STEP = 0.5

DIST_VARIANCE = 0.1
SPEED_VARIANCE = 0.1

ACCEL_PENALTY = 4
SPEED_PENALTY = 3

BUS_SPEED = 2

MAX_LOCAL_STEPS = 500

DISCOUNT = 0.98

INVALID_ACTION_PENALTY = 5

BUS_REMOVAL_PROB = 1

BUS_ARRIVAL_VARIANCE = 0.1

TIME_COST = 25
DISTANCE_COST = 25

# if this is not true, things get messed up
assert ACCEL_STEP == SPEED_STEP == DIST_STEP
