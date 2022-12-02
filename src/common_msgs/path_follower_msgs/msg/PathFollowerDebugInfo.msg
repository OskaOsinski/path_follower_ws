Header header
int8 path_received
int8 destination_reached
int8 path_lost
int8 brake_command

uint8 PREP_TO_DRIVE=0
uint8 WAIT_FOR_PATH_VAL=1
uint8 WAIT_FOR_GEAR=2
uint8 CAR_DRIVING=3
uint8 DEST_REACHED=4
uint8 PATH_CROSSED=5
uint8 RESET_STATE=6

int8 current_state
int8 previous_state
