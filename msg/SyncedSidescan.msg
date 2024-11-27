std_msgs/Header header
uint8   type          # Packet Type, 0xE2 = Sonar 8 Bit
uint32  time          # Ping time   [100 us]
uint8   frequency_id  # Freq Id, currently 0x00
int16   gain
uint16  decimation
float32 max_duration  # Max travel time of outermost bins (s)
float32 altitude # from DVL (m), should be positive
float32 max_r # in (m) accounted for sound speed
geometry_msgs/Vector3 pos # UTM Easting, Northing, Altitude in ENU convention
geometry_msgs/Vector3 angle # Roll, Pitch, Yaw (rad) in ENU convention

uint8[] port_channel
uint8[] starboard_channel
uint8[] extra_channel

float32[] port_channel_phase
float32[] starboard_channel_phase
float32[] port_channel_angle
float32[] starboard_channel_angle

bool[] port_valid_mask
bool[] starboard_valid_mask
float32[] port_tvu # vertical uncertainty
float32[] port_thu # horizontal uncertainty
float32[] starboard_tvu # vertical uncertainty
float32[] starboard_thu # horizontal uncertainty

sensor_msgs/PointCloud2 port_pointcloud # [along_track, across_track, z] in ENU convention
sensor_msgs/PointCloud2 starboard_pointcloud # [along_track, across_track, z] in ENU convention


