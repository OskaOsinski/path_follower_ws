bool valid
#clothoid parameters
float32 a
float32 b
float32 c
float32 d
float32 length
#line parameters
int32 line_id
int32 type #line type 0-solid, 1-dashed
int32 color #line color 0-white, 1-yellow
uint32[] x #x,y segment coordinates on image plane
uint32[] y
bool[604] valid_world
float32[604] x_world
float32[604] y_world

