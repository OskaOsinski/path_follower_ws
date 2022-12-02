bool valid              #entry is valid
int16 x 		#bounding box left upper corner x coordinate [pixels]
int16 y			#bounding box left upper corner y coordinate[pixels]
int16 w			#bounding box width [pixels]
int16 h			#bounding box height [pixels]
string label            #text type info
uint32 id               #object id
int32 type		#object type (0 - sign, 1-car, 2-pedestrian etc.)
int32 sub_type		#object sub type (for traffic signs it could be 0-stop sign, 1-speed limit etc., for cars it could be 0-passenger car, 1-truck etc. )
int32 speed_limit	#value of speed limit (5 = 5mph)
float32 confidence	#detection confidence [0-1]
float32 distance	#distance in meters to vehicle front axle

#sensor_msgs/Image image#Image data
float32 height		#Image dimensions in [m]
float32 width
float32 real_x		#Position in [m] 
float32 real_y
float32 real_z
float32 relative_velocity  # relative velocity of detected obstacle in [m/s]
float32 heading			   # heading of detected obstacle in [degrees]


