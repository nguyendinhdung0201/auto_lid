# 0: sonar_right 
# 1: lidar_front_right
# 2: sonar_front_center
# 3: lidar_front_center
# 4: lidar_front_left
# 5: sonar_left
# 6: sonar_rear_center

float32[7] data_sensor
float32 limit_forward_speed
float32 limit_backward_speed
float32 limit_spin_right_speed
float32 limit_spin_left_speed
float32[7] error