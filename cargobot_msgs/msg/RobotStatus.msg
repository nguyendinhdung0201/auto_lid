string robot_name
string version_name
uint8 status
uint8 operation_state   # 0 - Deactivate Mode 1 - Cruise Mode 2 - Developer Mode 3 - Delivery Mode
uint8 drive_state       # 0: RC mode 1: Tele-operation mode 2: AI (self-driving) mode 3: Joystick mode
uint8 delivery_state    # 0: Undefined state
                        # 1: Order Waiting
                        # 2: Order Received
                        # 3: Route available
                        # 4: Store coming
                        # 5: Store arrived
                        # 6: Package sent
                        # 7: Customer Coming
                        # 8: Customer Arrived
                        # 9: Customer Encountered
                        # 10: Package Delivered
                        # ---
                        # 101: Route unavailable
                        # 102: Customer not Encounter

uint8 cruise_state      # 0: Undefined state
                        # 1: Waiting
                        # 2: Received
                        # 3: Route available
                        # 4: Check point coming
                        # 5: Check point arrived
                        # ---
                        # 101: Route unavailable

uint8 lid_status        # 0: Open 1: Close

cargobot_msgs/BatteryStatus battery 
cargobot_msgs/SimStatus     sim_status
sensor_msgs/NavSatFix       gps
float32                     contermet_all
int8                        error
