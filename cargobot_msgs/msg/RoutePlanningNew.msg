# Route planning publish message
# ----------------------------------------------------------------
#
# order_state:
#   0: None
#   1: Waiting
#   2: Received
#
# route_check:
#   0: None
#   1: Unavailable
#   2: Available
# return_package:
#    true: return package to store
#    false: delivery package to customer
# destination:
#   0: None
#   1: Store
#   2: Customer
#   3: Gara
#
# moving_state:
#   0: None
#   1: Coming
#   2: Arrived
#
# confimation
#   0: None
#   1: Done
#   2: Encountered
#
#
# -------------------- Flow delivery -----------------------------
#
# order_state = Waiting
# order_state = Received
# route_check = Available
# destination = Store & moving_state = Coming
# destination = Store & moving_state = Arrived
# destination = Store & confimation = Done
# destination = Customer & moving_state = Coming
# destination = Customer & moving_state = Arrived
# confimation = Encountered
# confimation = Done
#
# ----------------------------------------------------------------
uint8 operation_mode
uint8 route_plan_ready
uint8 route_check
uint8 destination
uint8 moving_state
bool return_package 

uint8 gps_inaccurate
float32 gps_error

float32   enable_route_change_direct
uint8   too_wrong_direct
float32 angle_to_next_node
float32 angle_to_path
float32 distance_to_next_node
float32 angular_velocity_right
float32 angular_velocity_left
uint8 service
float32 distance_to_path
float32 width