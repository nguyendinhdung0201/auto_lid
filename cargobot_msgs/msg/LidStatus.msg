std_msgs/Header header
uint8 CLOSE = 0 #
uint8 OPEN = 1
uint8 RUN = 2
uint8 STUCK = 101
uint8 status
uint16 current_threshold
float32 current   # A
float32 voltage   # V
uint8 button_state
uint8 error

