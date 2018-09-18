import rospy
from smarc_msgs.msg import StringArray
from smarc_msgs.msg import SMTask

def add_string_argument(task, string_arg):
	task.action_arguments.append(StringArray([SMTask.STRING_TYPE, string_arg]))

def add_int_argument(task, int_arg):
	task.action_arguments.append(StringArray([SMTask.INT_TYPE, str(int_arg)]))

def add_float_argument(task, float_arg):
	task.action_arguments.append(StringArray([SMTask.FLOAT_TYPE, str(float_arg)]))

def add_time_argument(task, time_arg):
	task.action_arguments.append(StringArray([SMTask.TIME_TYPE, str(time_arg.to_sec())]))

def add_duration_argument(task, duration_arg):
	task.max_duration = rospy.Duration(duration_arg)

def add_bool_argument(task, bool_arg):
	task.action_arguments.append(StringArray([SMTask.BOOL_TYPE, str(bool_arg)]))

def add_pose_stamped_argument(task, pose_arg):
	task.action_arguments.append(StringArray([SMTask.POSE_STAMPED_TYPE,
	 pose_arg.header.frame_id,
	 str(pose_arg.pose.position.x),
	 str(pose_arg.pose.position.y),
	 str(pose_arg.pose.position.z),
	 str(pose_arg.pose.orientation.x),
	 str(pose_arg.pose.orientation.y),
	 str(pose_arg.pose.orientation.z),
	 str(pose_arg.pose.orientation.w)]))