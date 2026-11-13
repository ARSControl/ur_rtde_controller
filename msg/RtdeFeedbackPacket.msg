# One complete RTDE output package. Sequence is local to the collector session;
# it is not a robot packet counter and cannot establish source losslessness.
uint16 schema_version
string collector_session
uint64 sequence
float64 robot_timestamp
uint64 host_receive_ns
string host_clock_id
float64 requested_frequency_hz
string controller_version
uint8 protocol_version
uint8 recipe_id
string[] recipe_fields
float64[6] actual_q
float64[6] actual_qd
float64[6] target_qd
float64[6] actual_tcp_pose
float64[6] actual_tcp_speed
float64 speed_scaling
uint32 robot_status_bits
uint32 safety_status_bits
bool source_loss_known
