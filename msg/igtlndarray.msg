# ndarray msg headers
string name
uint8 scalar_type
uint8 dim
uint16[] size
# separate fields for different possible data types
int8[] data_int8
uint8[] data_uint8
int16[] data_int16
uint16[] data_uint16
int32[] data_int32
uint32[] data_uint32
float32[] data_float32
float64[] data_float64
# TODO complex data support