# Constant to represent no task 
#uint64 NO_TASK=0
# Constant for float type argument
string FLOAT_TYPE="____float____"
# Constant for int type argument
string INT_TYPE="____int____"
# Constant for ros time type argument
string TIME_TYPE="____time____"
# Constant for ros duration type argument
string DURATION_TYPE="____duration____"
# Constant for bool type
string BOOL_TYPE="____bool____"
# Constant for string type
string STRING_TYPE=____str____
# Constant for posestamped type
string POSE_STAMPED_TYPE=____pose____

# This holds the id of the task
uint64 task_id

# Termination condition WP: altitude. Set to zero if not used
float64 altitude

# Termination condition WP: depth. Set to zero if not used
float64 depth

# Termination condition WP: X. Set to zero if not used
float64 x

# Termination condition WP: Y. Set to zero if not used
float64 y

# Termination condition WP: Yaw. Set to zero if not used
float64 theta

# Action server topic
string action_topic

# Termination condition: maximum duration of the task. Set to zero if not used
duration max_duration

# Additional arguments to the action server call
smarc_msgs/StringArray[] action_arguments
