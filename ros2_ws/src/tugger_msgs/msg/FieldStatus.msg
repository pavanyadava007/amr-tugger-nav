# Status of the protective field monitor, published once per evaluated scan.
uint8 CLEAR=0
uint8 WARNING=1
uint8 PROTECTIVE_STOP=2

std_msgs/Header header
bool enabled                      # false = observe only, cmd_vel is passed through unchanged
uint8 state
uint8 field_set                   # index of the active speed band
float64 selection_speed           # max(|measured v|, |commanded v|) used to pick the field set
float64 protective_length         # field length ahead of the robot front edge [m]
float64 protective_width          # [m]
float64 warning_length            # [m]
float64 warning_width             # [m]
bool protective_violated          # a scan point is inside the protective field in this scan
bool warning_violated
geometry_msgs/Point nearest_violation   # nearest violating point, odom frame (valid if protective or warning violated)
uint32 protective_stops           # cumulative count of transitions into PROTECTIVE_STOP
uint32 warning_entries            # cumulative count of transitions into WARNING
uint32 protective_intrusions      # cumulative count of scans that start a protective violation (also counted when disabled)
float64 eval_time_us              # wall time to evaluate this scan [microseconds]
