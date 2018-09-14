# Constant to represent no task 
uint64 NO_TASK=0
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

# This holds the id of the task
uint64 task_id

# WP altitude 
uint64 altitude

# WP Depth
uint64 dept

# WP X
uint64 x

# WP Y
uint64 y

# WP Yaw
uint64 theta

# Type of state in the state machine [INIT, ACTION, END]
string state

# The action server to be called to trigger the task
string action

# Action server topic
string action_topic

# The maximum duration of the task
duration max_duration

# The scheduler execution time
time execution_time

# Priority of a task - higher number = higher priority
uint64 priority

# Additional arguments to the action server call
smarc_msgs/StringPair[] arguments
