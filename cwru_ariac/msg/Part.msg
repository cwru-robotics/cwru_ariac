string name
int32 id
int8 traceable
int8 location
geometry_msgs/PoseStamped pose
geometry_msgs/Vector3 linear
float64 priority

# location list
int8 UNASSIGNED=1 # unconfirmed location
int8 CAMERA=10 # before location separation
int8 CONVEYOR=11
int8 GROUND=12
int8 AGV=20       # on any of AGVs
int8 AGV1=21
int8 AGV2=22
int8 BIN=30       # on any of bins
int8 BIN1=31
int8 BIN2=32
int8 BIN3=33
int8 BIN4=34
int8 BIN5=35
int8 BIN6=36
int8 BIN7=37
int8 BIN8=38