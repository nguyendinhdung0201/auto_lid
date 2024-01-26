# Message published from obstacle avoidance node
#
# Dodge type:
#   DODGE_NO_ACTION = 0
#   DODGE_STOP = 1
#   DODGE_EDGE_RIGHT = 2
#   DODGE_EDGE_LEFT = 3
#   DODGE_REVERSE = 4
#   DODGE_STUCK = 5
#   DODGE_50_LEFT = 501
#   DODGE_50_RIGHT = 503
#   DODGE_100_LEFT = 1001
#   DODGE_100_RIGHT = 1003
#   DODGE_200_LEFT = 2001
#   DODGE_200_RIGHT = 2003

uint8 obs_avoid_ready
float32 enable_dodge
uint8 dodge_stuck
uint16 dodge_type
uint8 dodge_phase
geometry_msgs/Twist obs_avoid_vel