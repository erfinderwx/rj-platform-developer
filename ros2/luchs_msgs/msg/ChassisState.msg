# Luchs chassis state as reported by the chassis, plus the driver's own link status.
# Wheel arrays are ordered front-left, front-right, rear-left, rear-right.
# Fields in the "LCI" block are filled by chassis that speak LCI (Luchs B); they keep their
# defaults on the Luchs A protocol. Check lci_ok before using them.

std_msgs/Header header

uint8 CONTROL_UNKNOWN=0
uint8 CONTROL_REMOTE=1        # remote-control transmitter is driving
uint8 CONTROL_HOST=2          # host computer (this driver) is driving
uint8 control_source

uint8 MODEL_DIFFERENTIAL=1
uint8 MODEL_FRONT_ACKERMANN=2
uint8 MODEL_FOUR_WHEEL_STEER=3
uint8 MODEL_DUAL_ACKERMANN=4
uint8 motion_model            # as reported by the chassis

uint8 GEAR_DISABLED=0
uint8 GEAR_PARK=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 GEAR_DRIVE=4
uint8 gear

geometry_msgs/Twist twist     # body twist measured by the chassis at base_link (m/s, rad/s)
float64 front_steer_angle     # rad, axle (bicycle) angle, Ackermann builds
float64 rear_steer_angle      # rad
float64[4] wheel_steer_angle  # rad, four-wheel-steer build
float64[4] wheel_speed        # rad/s

uint8 fault_code              # 0 = no fault, see fault_text
string fault_text
uint32 motor_fault_high
uint32 motor_fault_low

uint8 turn_signal             # lamp feedback: 0 off, 1 left, 2 right
bool reverse_lamp
bool fault_lamp
uint8 light_rain_raw          # light/rain sensor builds: 0x00 dark-dry, 0x01 dark-rain, 0x10 light-dry, 0x11 light-rain

# ---- LCI (Luchs B) ------------------------------------------------------------------------
bool lci_ok                   # LCI chassis-state frame (0x5BD) received recently

uint8 MODE_MAINTENANCE=0
uint8 MODE_REMOTE_CONTROL=1
uint8 MODE_AUTOMATIC=2
uint8 operating_mode

uint8 estop_source_bits       # bit 0 onboard L, 1 onboard R, 2 RC, 3 host, 4 bumper F, 5 bumper R, 6 external loop, 7 heartbeat timeout
string estop_source_text

uint8 FAULT_NONE=0
uint8 FAULT_WARNING=1
uint8 FAULT_DERATED=2
uint8 FAULT_STOP=3
uint8 fault_level

bool rc_online
bool rc_enabled
uint8 rc_link                 # 0 good, 1 weak, 2 lost, 3 not fitted
bool four_wheel_drive
bool park_brake_engaged
float64 active_speed_limit    # m/s, effective cap after all limits

# live limits the chassis enforces now (0x5B0)
float64 max_vx_forward        # m/s
float64 max_vx_reverse        # m/s
float64 max_steer_angle       # rad, at the current speed
uint8 limit_reason_bits       # bit 0 slope/roll, 1 temperature, 2 battery, 3 fault derate, 4 host speed limit, 5 speed-dependent steering, 6 maintenance/RC, 7 other
string limit_reason_text
bool speed_clipped
bool steer_clipped

# Luchs B brake, parking brake, steering actuators, temperatures (0x5C0, 0x5C1)
float64 brake_pressure        # MPa
uint8 epb_state               # 0 released, 1 applying, 2 applied, 3 releasing, 4 fault
uint8 eps_front_state         # 0 ok, 1 warning, 2 degraded, 3 failed
uint8 eps_rear_state
uint8 brake_state
float64[4] motor_temperature       # deg C
float64[4] controller_temperature  # deg C

# identification (0x5BF)
uint8 model_id                # 0x0A..0x0D Luchs A builds, 0x1B Luchs B
string model_text
string firmware_version
string lci_version

# driver status
bool feedback_ok              # chassis feedback received within feedback_timeout
bool command_active           # a command was received within cmd_timeout
bool command_limited          # the last command was reduced by a speed or steering limit
uint8 commanded_motion_model
geometry_msgs/Twist commanded_twist
uint8 heartbeat_counter       # LCI: last AliveCounter sent
bool estop_requested          # host requests in the heartbeat
bool park_requested
bool payload_power_1
bool payload_power_2
float64 host_speed_limit      # m/s, 0 = chassis default
bool model_mismatch           # the chassis identifies as a different model than configured
