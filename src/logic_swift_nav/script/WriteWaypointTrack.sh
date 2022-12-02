#!/bin/bash

ROS_PACKAGE=$1

echo "=== Starting waypoint writer script ==="
echo "Ros package: "$ROS_PACKAGE

source /opt/ros/kinetic/setup.bash
cd $ROS_PACKAGE
cd ../..
source devel/setup.bash

roslaunch rqt_swift_nav gps_waypoint_writer.launch

cd $ROS_PACKAGE/launch

REF_LAT_PARAM=`rosparam get /utm_converter/ref_lat_param`
REF_LONG_PARAM=`rosparam get /utm_converter/ref_long_param`
WAYPOINT_FILE=`rosparam get /gps_waypoint_writer/waypoint_file_path`
REFPOINT_FILE=`rosparam get /gps_waypoint_writer/reference_file_path`

cat <<EOF > gps_waypoint_navigation_kia.launch
<?xml version="1.0" ?>

<launch>

  <!-- Dataspeed launch file-->
  <include file="\$(find kia_control)/launch/kia_oscc_system.launch"/>
  <node name="rqt_gui" pkg="rqt_gui" type="rqt_gui" args="--perspective-file \$(find kia_control)/perspective/oscc_cmd.perspective" output="screen"/>

  <!-- GPS launch file -->
  <include file="\$(find swift_nav_sbp_udp)/launch/udpSwiftNavReceiver.launch" />

  <!-- Navigation nodes -->
  <node pkg="gps_waypoint_navigation" type="utm_converter" name="utm_converter" output="screen">
  <param name="ref_lat_param" value="$REF_LAT_PARAM" />
  <param name="ref_long_param" value="$REF_LONG_PARAM" />
  </node>

  <node pkg="gps_waypoint_navigation" type="waypoint_handler" name="waypoint_handler" output="screen">
    <param name="waypoint_file_path" type="str" value="$WAYPOINT_FILE" />
  </node>

  <node pkg="waypointsnavigationcontroller_ab_oa_lookahead_touchscreen" type="waypointsnavigationcontroller_ab_oa_lookahead_touchscreen_node" name="waypointsnavigationcontroller_ab_oa_lookahead_touchscreen_node" output="screen">
  </node>

  <!-- Visualization -->
  <node pkg="rviz" type="rviz" name="sim_project_rviz" respawn="true" args="-d \$(find gps_waypoint_navigation)/rviz/gps_navigation.rviz"/>

</launch>
EOF
