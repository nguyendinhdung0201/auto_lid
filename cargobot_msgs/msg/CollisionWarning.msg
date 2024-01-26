# Collision warning system state and collision_warning_speed
# collision_warning_state:
#   0: Objects too close --> Stop
#   1: Objects near robot --> Slowdown
#   2: No objects in the way --> Normal speed
#
uint8 collision_warning_state
float32 collision_avoid_speed

# Distance from sensor to object in meter
#
float32 lidar_front
float32 lidar_left
float32 lidar_right
float32 lidar_back
float32 sonar_front