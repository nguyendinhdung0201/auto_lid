
uint8 WAITING     = 1
uint8 RECEIVED    = 2
uint8 ROUTE_AVAILABLE = 3
uint8 CHECK_POINT_COMING  = 4
uint8 CHECK_POINT_ARRIVED  = 5
uint8 ROUTE_UNAVAILABLE = 101

uint8 cruise_state_monitor

# cruise_state_monitor:
#   0:   Undefined state
#   1:   Waiting 
#   2:   Received
#   3:   Route Available
#   4:   Check point Coming
#   5:   Check point Arrived
#   101: Route Unavailable 