# ----------------------------------------------------------------
#
# operation_delivery_mode:
#   0: Disable Delivery Mode
#   1: Enable Delivery Mode
#
# drive_tele_mode
#   0: Disable Drive Teleoperation Mode
#   1: Enable Drive Teleoperation Mode
#
# ----------------------------------------------------------------
#
# delivery_order_state:
#   0: None
#   1: Waiting
#   2: Received
# For wedding
#   10: Go to couple
#   11: Back home
#
# delivery_confimation
#   0: None
#   1: Done
#   2: Encountered
#
# ----------------------------------------------------------------
#
# store_location:
#   GPS lat long of store
#
# customer_location:
#   GPS lat long of customer
#
# ----------------------------------------------------------------
#
# tele_cmd_vel:
#   Command velocity from Tele-operator
#
# ----------------------------------------------------------------
#
# open_lid_cmd:
#   0: No action
#   1: Open lid
#
# ----------------------------------------------------------------

uint8 DEACTIVATE = 0
uint8 DELIVERY = 1
uint8 CRUISE = 2

uint8 operation_mode
                    # 0: Enable Deactivate Mode
                    # 1: Enable Delivery Mode 
                    # 2: Enable Cruise Mode
uint8 drive_tele_mode
                    
uint8 server_cmd_state 
                        # 0: None
                        # 1: Waiting
                        # 2: Received

uint8 NONE = 0
uint8 DONE = 1
uint8 ENCOUNTERED = 2
uint8 ARRIVED = 3

uint8 confirmation

geometry_msgs/Vector3 store_location           # GPS location of store / Check point if  Enable Cruise Mode
geometry_msgs/Vector3 customer_location

geometry_msgs/Twist tele_cmd_vel

uint8 open_lid_cmd                              # 0: No action 1: Open lid
