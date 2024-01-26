# Convert from mavros/RCIn to meanful message
# Operation Mode:
#   operation_mode = 0 - Deactivate Mode
#   operation_mode = 1 - Neutral Mode
#   operation_mode = 2 - Developer Mode
#
#

uint8 e_stop
uint8 operation_mode_cmd
uint8 enable_rc_drive_mode
uint8 enable_collect_data
uint8 open_lid_cmd
float32 direct_high_cmd
geometry_msgs/Twist remote_vel_cmd
