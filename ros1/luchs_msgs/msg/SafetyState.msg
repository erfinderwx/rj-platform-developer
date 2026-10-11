# Protection inputs of the Luchs chassis.

std_msgs/Header header

bool estop_left
bool estop_right
bool bumper_front
bool bumper_rear

uint8 ULTRASONIC_NONE=0
uint8 ULTRASONIC_LEFT=1
uint8 ULTRASONIC_RIGHT=2
uint8 ULTRASONIC_FRONT=3
uint8 ULTRASONIC_REAR=4
uint8 ultrasonic_trigger

float32[4] ultrasonic_range   # m, order front, rear, left, right
uint8 fault_code
string fault_text
