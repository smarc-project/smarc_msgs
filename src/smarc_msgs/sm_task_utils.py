
# Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
