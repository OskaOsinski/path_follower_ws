uint8 DW_PATH_POSITION_TYPE_INVALID                   = 0
uint8 DW_PATH_POSITION_TYPE_UNDEFINED                 = 1
uint8 DW_PATH_POSITION_TYPE_EGO                       = 2
uint8 DW_PATH_POSITION_TYPE_ADJACENT_LEFT             = 3
uint8 DW_PATH_POSITION_TYPE_ADJACENT_RIGHT            = 4
uint8 DW_PATH_POSITION_TYPE_EGO_FORK_LEFT             = 5
uint8 DW_PATH_POSITION_TYPE_EGO_FORK_RIGHT            = 6
uint8 DW_PATH_POSITION_TYPE_ADJACENT_LEFT_FORK_LEFT   = 7
uint8 DW_PATH_POSITION_TYPE_ADJACENT_RIGHT_FORK_RIGHT = 8

uint8 DW_PATH_ATTRIBUTE_TYPE_UNDEFINED        = 0
uint8 DW_PATH_ATTRIBUTE_TYPE_OPPOSITE_TRAFFIC = 1


uint8 positionType
uint8 pathAttributeType

uint32 numPoints
float32 confidence
float32 pathAttributeConfidence

float32[] confidenceValues
geometry_msgs/Point[] leftEdgePoints
geometry_msgs/Point[] rightEdgePoints
geometry_msgs/Point[] centerRailPoints
float32[] pathWidth
float32[] pathAngle