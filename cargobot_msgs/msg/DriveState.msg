

uint8 drive_tele_ready      # 0: Not Ready 1: Ready
uint8 drive_joy_ready       # 0: Not Ready 1: Ready
uint8 drive_ap_ready        # 0: Not Ready 1: Ready
#=====
uint8 RC_MODE       = 0
uint8 TELE_MODE     = 1
uint8 AP_MODE       = 2
uint8 JOYSTICK_MODE = 3
uint8 drive_mode_state
#=====
geometry_msgs/Twist drive_velocity