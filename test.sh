#!/bin/bash

# Arguments:
#   1: Distro to test

hokuyo_model="PackageSet {
  CatkinPackage hokuyo_node {
    Artifact hokuyo_node {
      Node { name hokuyo_node
        Publishers {
          Publisher { name 'scan' message 'sensor_msgs.LaserScan'}}}
}}}"

if [ "${1}" = "melodic" ]; then
  ./haros_runner.sh hokuyo_node hokuyo_node node . /root/ws https://github.com/ros-drivers/hokuyo_node
  if [ "$(echo "$hokuyo_model" | sed -E 's,\\t|\\r|\\n,,g')" != "$(cat /root/ws/hokuyo_node.ros | sed -E 's,\\t|\\r|\\n,,g')" ] ;then
    echo "Test failed; the auto-generated model is different to the expected one:"
    echo "$hokuyo_model"
    exit 1
  fi
fi

s300_model="PackageSet {
  CatkinPackage cob_sick_s300 {
    Artifact cob_sick_s300 {
      Node { name cob_sick_s300
        Publishers {
          Publisher { name '/diagnostics' message 'diagnostic_msgs.DiagnosticArray'},
          Publisher { name 'scan_standby' message 'std_msgs.Bool'},
          Publisher { name 'scan' message 'sensor_msgs.LaserScan'}}}
}}}"

if [ "${1}" = "noetic" ]; then
  ./haros_runner.sh cob_sick_s300 cob_sick_s300 node . /root/ws https://github.com/ipa320/cob_driver
  if [ "$(echo "$s300_model" | tr -d '[:space:]')" != "$(cat /root/ws/cob_sick_s300.ros | tr -d '[:space:]')" ] ;then
      :
  else
    echo "Test failed; the auto-generated model is different to the expected one:"
    echo "$s300_model"
    exit 1
  fi
fi

safetyscanner_model="PackageSet {
  CatkinPackage sick_safetyscanners2 {
    Artifact sick_safetyscanners2_node {
      Node { name sick_safetyscanners2_node
        ServiceServers {
          ServiceServer { name 'field_data' service 'sick_safetyscanners2_interfaces.FieldData'}}
        Publishers {
          Publisher { name 'scan' message 'sensor_msgs.LaserScan'},
          Publisher { name 'output_paths' message 'sick_safetyscanners2_interfaces.OutputPaths'},
          Publisher { name 'raw_data' message 'sick_safetyscanners2_interfaces.RawMicroScanData'},
          Publisher { name 'extended_scan' message 'sick_safetyscanners2_interfaces.ExtendedLaserScan'}}}
}}}"

if [ "${1}" = "foxy" ]; then
  ./haros_runner.sh sick_safetyscanners2 sick_safetyscanners2_node node . /root/ws  https://github.com/SICKAG/sick_safetyscanners2
  if [ "$safetyscanner_model" == "$(cat /root/ws/sick_safetyscanners2_node.ros)" ] ;then
      :
  else
    echo "Test failed; the auto-generated model is different to the expected one:"
    echo "$safetyscanner_model"
    exit 1
  fi
fi
