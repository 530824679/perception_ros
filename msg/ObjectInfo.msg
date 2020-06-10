uint16                       id                 # The No. of object
uint8                        type 		        # The category of object
# unknow     0
# pedestrain 1
# motor      2
# car        3
# truck      4

uint8                        yaw                # The orientation angle of object
uint8                        confidence         # The confidence of object
uint16                       height             # The height of object
uint16                       width              # The width of object
uint16                       length             # The length of object
int16                        distance_xv        # The longitudinal distance of object to ego vehicle coordinate
int16                        distance_yv        # The lateral distance of object to ego vehicle coordinate
int16                        velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate
int16                        velocity_yv        # The lateral velocity of object to ego vehicle coordinate
int16                        accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate
int16                        accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate
uint8                        motion_state       # The motion status of object
# unknow     0
# moving     1
# stationary 2



