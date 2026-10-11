# Lamp request for the Luchs chassis (builds with host-controlled lamps).
uint8 TURN_OFF=0
uint8 TURN_LEFT=1
uint8 TURN_RIGHT=2
uint8 TURN_HAZARD=3           # LCI
uint8 turn_signal
bool reverse_lamp
bool fault_lamp

uint8 HEADLIGHT_OFF=0
uint8 HEADLIGHT_LOW=1
uint8 HEADLIGHT_HIGH=2
uint8 headlight               # LCI; Luchs B VCU interface: on / off
bool horn                     # LCI
bool beacon                   # LCI, where fitted
