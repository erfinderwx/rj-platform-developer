# Advanced command: choose the motion model and, optionally, explicit steering angles.
# For normal use publish geometry_msgs/Twist on cmd_vel instead.
# The command is subject to the same watchdog as cmd_vel (re-send at >= 1 / cmd_timeout Hz).

std_msgs/Header header

uint8 MODEL_KEEP=0
uint8 MODEL_DIFFERENTIAL=1
uint8 MODEL_FRONT_ACKERMANN=2
uint8 MODEL_FOUR_WHEEL_STEER=3
uint8 MODEL_DUAL_ACKERMANN=4
uint8 motion_model

geometry_msgs/Twist twist

bool use_explicit_angles      # false: angles are computed from the twist
float64 front_steer_angle     # rad
float64 rear_steer_angle      # rad
float64[4] wheel_steer_angle  # rad, front-left, front-right, rear-left, rear-right
