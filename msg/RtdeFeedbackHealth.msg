# Collector-only counters. ROS/recorder loss and robot-side source loss are
# not determined by local sequence continuity or these counters.
uint16 schema_version
string collector_session
string host_clock_id
uint64 host_time_ns
string connection_state
string detail
uint64 packets_received
uint64 packets_published
uint64 local_queue_dropped
uint64 publication_failures
uint64 timestamp_regressions
uint64 source_gap_observations
uint64 malformed_packets
uint64 transport_errors
uint64 reconnects
uint64 last_sequence
uint64 pending_queue
bool source_loss_known
