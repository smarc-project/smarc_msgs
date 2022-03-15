^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smarc_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2022-03-15)
------------------
* Merge pull request `#25 <https://github.com/smarc-project/smarc_msgs/issues/25>`_ from KKalem/noetic-devel
  Added a string field for action feedback to GotoWaypointAction
* Added a string field for action feedback to GotoWaypointAction
* Merge pull request `#24 <https://github.com/smarc-project/smarc_msgs/issues/24>`_ from KKalem/noetic-devel
  Added GotoWaypoint message
* Added GotoWaypoint message that mimics the action for unplanned waypoint commanding
* Update release.yml
* Merge pull request `#22 <https://github.com/smarc-project/smarc_msgs/issues/22>`_ from smarc-project/add-dr-convenience-topics
  Add DR convenience topics
* Update README.rst
* Update README.rst
* Merge pull request `#21 <https://github.com/smarc-project/smarc_msgs/issues/21>`_ from smarc-project/test-noetic
  Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Merge pull request `#20 <https://github.com/smarc-project/smarc_msgs/issues/20>`_ from nilsbore/test_release
  Release build
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Update package.xml
* Update package.xml
* Update release.yml
* Update release.yml
* Update release.yml
* Update release.yml
* Create release.yml
* Merge pull request `#18 <https://github.com/smarc-project/smarc_msgs/issues/18>`_ from smarc-project/nilsbore-patch-1
  Update GotoWaypoint.action
* Update GotoWaypoint.action
* Update README.rst
* Merge pull request `#15 <https://github.com/smarc-project/smarc_msgs/issues/15>`_ from smarc-project/nilsbore-patch-1
  Update GotoWaypoint.action
* Update GotoWaypoint.action
* Merge pull request `#14 <https://github.com/smarc-project/smarc_msgs/issues/14>`_ from smarc-project/ned_enu
  Added support for translating lat lon odometry
* added support for translating lat lon odometry
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Added NEDENU conversion service
* Update README.rst
* Update README.rst
* Merge pull request `#12 <https://github.com/smarc-project/smarc_msgs/issues/12>`_ from smarc-project/interface
  Proposal for a common SMARC AUV ROS interface
* Update README.rst
* Update README.rst
* Update CMakeLists.txt
* Merge pull request `#13 <https://github.com/smarc-project/smarc_msgs/issues/13>`_ from smarc-project/noetic-devel
  Create SensorStatus.msg
* Create SensorStatus.msg
* Delete SidescanStamped.msg
* Delete SidescanPositioned.msg
* Update CMakeLists.txt
* Update Sidescan.msg
* Update CMakeLists.txt
* Rename CTDFeedback.msg to CTD.msg
* Update GotoWaypoint.action
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Added DVL message from uuv_simulator
* Update README.rst
* Update README.rst
* Update CMakeLists.txt
* Create ControllerStatus.msg
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update CMakeLists.txt
* Create GotoWaypoint.action
* Added service definitions for UTM/lat lon conversion
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update README.rst
* Update and rename README.md to README.rst
* Update SidescanPositioned.msg
* Create main.yml
* Merge pull request `#10 <https://github.com/smarc-project/smarc_msgs/issues/10>`_ from Jollerprutt/add_sss_msgs
  add Sidescan messages
* add Sidescan messages
* Merge pull request `#8 <https://github.com/smarc-project/smarc_msgs/issues/8>`_ from Jollerprutt/master
  generate messages for DualThruster RPM/Feedback
* generate messages for DualThruster RPM/Feedback
* Merge pull request `#7 <https://github.com/smarc-project/smarc_msgs/issues/7>`_ from Jollerprutt/master
  add CTDFeedback msg
* add CTDFeedback msg
* Merge pull request `#6 <https://github.com/smarc-project/smarc_msgs/issues/6>`_ from smarc-project/new_msgs
  Added some new messages for Lolo as per discussion
* Added some new messages
* Added some new messages for Lolo as per discussion
* This should just be nacho
* Added executions tatus message
* Merge pull request `#5 <https://github.com/smarc-project/smarc_msgs/issues/5>`_ from ignaciotb/working_branch
  New srv to add several tasks at the once
* New srv to add several tasks at the once
* Merge pull request `#4 <https://github.com/smarc-project/smarc_msgs/issues/4>`_ from ignaciotb/working_branch
  New def of SMTask.msg based on rospy_message_converter and some cleaning
* New def of SMTask.msg and some cleaning
* Merge pull request `#3 <https://github.com/smarc-project/smarc_msgs/issues/3>`_ from ignaciotb/working_branch
  Fixed type of waypoint members and cleaned up SMTask.msg
* Removed action result from SMTask msg definition
* Fixed type of waypoint members and cleaned up SMTask.msg
* Merged cmake
* Modified setup.py for scripts
* Missing some cleanup for SMTask.msg. Moved folders for importing python modules
* Merge remote-tracking branch 'origin/master'
* Created msg and srv for smach state machine
* Added an empty action definition
* Added an empty action definition
* Merge pull request `#1 <https://github.com/smarc-project/smarc_msgs/issues/1>`_ from KKalem/master
  added comms_msg for inter-agent communications. Looking good!
* Merge pull request `#1 <https://github.com/smarc-project/smarc_msgs/issues/1>`_ from KKalem/master
  added comms_msg for inter-agent communications. Looking good!
* renamed comms_msg to CommsMessage
* renamed comms_msg to CommsMessage
* added comms_msg for inter-agent communications
* added comms_msg for inter-agent communications
* Added a message type to check if costmap cell is occupied
* Added a message type to check if costmap cell is occupied
* Added the basic structure
* Added the basic structure
* Initial commit
* Initial commit
* Contributors: Carl Ljung, Nacho, Nils Bore, Ozer Ozkahraman, Özer Özkahraman
