=======================
SMARC AUV ROS Interface
=======================

.. contents:: Table of Contents
   :depth: 2

Background
==========

The aim of the SMARC ROS interface is to define a common interface that can be implemented across a range of AUVs. This enables the implementation of a shared set of components on top of these interfaces. For example, our goal is to share at least the following components:

  * Controllers, when actuation is the same
  * Planner interface (behavior tree in our case)
  * Mission planning interface (Neptus in our case)
  * Sensor interfaces, enabling shared inspection tools
  * GUI

It should be noted that the interface defined here might only be a subset of the full interface on the vehicle ROS system. Reasons for adding more interfaces might be differentiated actuators, which can not be shared across vehicles, or that some vehicle provides more sensors or other sources of information.

Preliminaries
=============

Namespaces
----------

We will refer to the global vehicle namespace as “/vehicle”. On the vehicles, this is replaced by the name of the vehicle, and may be e.g. “/sam” or “/lolo”. All topics on the vehicles should be under this namespace. The rationale for this is mainly that we can then run multiple vehicle systems within one simulator session, without topic names colliding.

/vehicle/core -  contains the sensors and actuators that are always available on the vehicle, also when no controllers or other higher-level functionality is running. It also includes basic control interfaces such as abort functionality
/vehicle/ctrl - contains topics and nodes relating to actuator controllers (e.g. depth, heading, altitude control)
/vehicle/dr - contains interfaces related to dead reckoning
/vehicle/payload - contains sensors and actuators that are not always available on all versions of all vehicles, such as sidescan, multi-beam, cameras etc.

The rationale for dividing everything into high-level namespaces is that this makes it easier to debug the system with ROS tools. For example, the rqt_graph presents a visual overview of topics and tools organized by namespaces, and allows collapsing the ones we are not interested in. This allows debugging of the subparts, rather than the whole system.

Repos
-----

This document defines interfaces based on messages in the standard ROS package repositories, and the smarc_msgs repo. Any vehicle-specific messages (outside the scope of this document) should be defined in separate packages. The rationale for this division is that the smarc_msgs repo together with this document (or future versions thereof) might be used as a specification of the shared interface.

Parameters
==========

The UTM zone parameters are used in dead reckoning interfaces and the planning (Neptus and behavior tree) interfaces. We define the interactions with our coordinate system in the TF section. Note that UTM zone should never be automatically calculated from GPS position but instead by looking at these parameters. The rationale is that this enables us to cross UTM zones while keeping the coordinate system. The UTM zones are defined in such a way as too allowing use even outside the bounds.

  * UTM Zone - int as /utm_zone, should be set at startup, the UTM longitude zone to use throughout the mission
  * UTM band - string as /utm_band, should be set at startup, the UTM latitude band to use throughout the mission
  
Topic interfaces
================

Core interface
--------------

Core sensor interfaces

The base set of sensors are all under the /vehicle/core namespace. They are all publishers.

  * IMU - sensor_msgs/Imu on /vehicle/core/imu
  * Pressure sensor - sensor_msgs/FluidPressure on /vehicle/core/pressure
  * GPS - sensor_msgs/NavSatFix on /vehicle/core/gps
  * Compass - sensor_msgs/MagneticField on /vehicle/core/compass
  * DVL - smarc_msgs/DVL (copied from cola2_msgs/DVL)
  * Leak - smarc_msgs/Leak on /vehicle/core/leak
  * Battery - sensor_msgs/BatteryState on /vehicle/core/battery

Core actuator interface

Publishers

  * RPM - smarc_msgs/ThrusterRPM on /vehicle/core/thruster{N}_cmd, where N signifies the number of the thruster. Thrusters are numbered either left-to-right or front-to-back, or both, depending on the configuration
  * Feedbacks
  * Thruster feedback - smarc_msgs/ThrusterFeedback on /vehicle/core/thruster{N}_fb

Core system interfaces

  * Abort - std_msgs/Empty on /vehicle/core/abort, aborts current mission, vehicle should surface by itself, with no more control from ROS system

Near future extensions
Then there are also a few preliminary ideas about how to combine the VBS and centre of gravity control

Publishers

  * VBS - smarc_msgs/PercentStamped on /vehicle/core/vbs_cmd
  * LCG - to be decided
  * TCG - to be decided

Subscribers

  * VBS feedback - smarc_msgs/PercentStamped on /vehicle/core/vbs_fb
  * LCG feedback
  * TCG feedback

Controller interfaces
---------------------

All controllers reside in the /vehicle/ctrl namespace.

Basic controller topics

  * Heading - std_msgs/Float64 on /vehicle/ctrl/yaw_setpoint
  * Depth - std_msgs/Float64 on /vehicle/ctrl/depth_setpoint
  * Altitude - std_msgs/Float64 on /vehicle/ctrl/alt_setpoint
  * Speed - std_msgs/Float64 on /vehicle/ctrl/speed_setpoint
  * Pitch - std_msgs/Float64 on /vehicle/ctrl/pitch_setpoint
  * Roll - std_msgs/Float64 on /vehicle/ctrl/roll_setpoint

Basic controller services

  * Toggle heading ctrl - std_srvs/SetBool on /vehicle/ctrl/toggle_heading_ctrl
  * Toggle depth ctrl - std_srvs/SetBool on /vehicle/ctrl/toggle_depth_ctrl
  * Toggle altitude ctrl - std_srvs/SetBool on /vehicle/ctrl/toggle_altitude_ctrl
  * Toggle speed ctrl - std_srvs/SetBool on /vehicle/ctrl/toggle_speed_ctrl
  * Toggle pitch ctrl - std_srvs/SetBool on /vehicle/ctrl/toggle_pitch_ctrl
  * Toggle roll ctrl - std_srvs/SetBool on /vehicle/ctrl/toggle_roll_ctrl

Planners (advanced controllers)
-------------------------------

  * Go to waypoint - smarc_msgs/WaypointAction on /vehicle/ctrl/goto_waypoint

Dead reckoning
--------------

All dead reckoning topics and nodes reside within the /vehicle/dr namespace

Topics

  * Dead reckoning odometry (poses, velocities and uncertainties) - nav_msgs/Odometry on topic /vehicle/dr/odom

TF
--

The TF tree can be constructed from the /vehicle/dr/odom topic. If /vehicle/dr/odom is present, it is therefore not necessary to provide the TF tree, although some implementations provide both as one package.

  * Shared UTM frame - “utm”
  * Shared local map frame - “map”
  * Vehicle odometry frame “vehicle/odom”
  * Vehicle origin frame “vehicle/base_link”
  * Frames for sensors, as referenced in the header stamp/frame_id messages. E.g. “vehicle/imu_link”

The resulting TF tree has the structure “utm -> map -> vehicle/odom -> vehicle/base_link -> vehicle/imu_link”. Note that “imu_link” can be exchanged for any other frame on the vehicle.

The “utm -> vehicle/base_link” is the most interesting transform as it provides the vehicle pose in the coordinate system of the local UTM zone. Which UTM zone this is referring to is given by the /utm_zone and /utm_band parameters, which are set at start-up.

Payloads
--------

These are all optional. They do not need to be published to fulfill the ROS interface specification, but if they are, the should be available in the form presented here.

Payload sensor topics

  * Sidescan - smarc_msgs/SideScan on topic /vehicle/payload/sidescan
  * CTD - smarc_msgs/CTD on topic /vehicle/payload/ctd

Payload sensor services

  * Enable/disable sidescan - std_srvs/SetBool on /vehicle/payload/toggle_sidescan - send true to turn on and false to turn off, returns true if successful

Tools aiding implementation
===========================

These tools are not part of the vehicle interface specification since they are not required to be implemented for each vehicle. Nodes already exist that implement them, you might expect them to be running on the system, and they may be used to implement the vehicle interface.

tf_lat_lon package
------------------
Apart from the services, the tf_lat_lon package also offers a c++ library for doing conversions between tf and latitude/longitude.
Services (always there)

  * Lat lon to UTM conversion - smarc_msgs/LatLonToUTM on /vehicle/dr/lat_lon_to_utm
  * UTM to lat lon conversion - smarc_msgs/UTMToLatLon on /vehicle/dr/utm_to_lat_lon

Controllers
-----------

For each controller specified in the controller section, we may alternatively implement them to require setpoints at a certain frequency to keep going. In order to translate it to the interface above, we offer a node that repeats a setpoint at a certain frequency depending on if the service has been called to activate the controller. In the specification below, {target} may be either of heading, depth, altitude, speed, pitch or roll. Since they all take in std_msgs/Float64, we can just launch multiple instances of the same node, one for every controlled target.

Nodes

  * control_throttle_service - offers service /vehicle/ctrl/toggle_{target}_ctrl to start and stop publishing to /vehicle/ctrl/{target}_setpoint_freq. Listens to /vehicle/ctrl/{target}_setpoint and republishes at a set frequency if started
