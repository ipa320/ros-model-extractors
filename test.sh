#!/bin/bash

# Arguments:
#   1: Distro to test

RED='\033[0;31m'
GREEN='\033[0;32m'

test_model="PackageSet {
  CatkinPackage test_pkg {
    Artifact test_node {
      Node { name test_node
        ServiceServers {
          ServiceServer { name 'setBool' service 'std_srvs.SetBool'}}
        ServiceClients {
          ServiceClient { name 'init' service 'std_srvs.Trigger'}}
        Publishers {
          Publisher { name 'scan' message 'sensor_msgs.LaserScan'}}
        Subscribers {
          Subscriber { name 'power_state' message 'sensor_msgs.BatteryState'}}}
}}}"

./haros_runner.sh test_pkg test_node node . /root/ws https://github.com/ipa-nhg/test_ros_code_extractor "${1}"


if [ "$(echo "$test_model" | sed -E 's,\\t|\\r|\\n,,g')" != "$(cat /root/ws/test_node.ros | sed -E 's,\\t|\\r|\\n,,g')" ] ;then
  echo -e "${RED} [TEST] EXTRACTION FAILS"
  echo -e "${RED} EXTRACTED AND EXPECTED MODEL DON'T MATCH"
  echo "The expected model is:"
  echo "~~~"
  echo "$test_model"
  echo "~~~"
  exit 1
else
  echo -e "${GREEN} [TEST] BUILD SUCCESSFUL"
  echo -e "${GREEN} [TEST] EXTRACTION SUCCESSFUL"
  echo -e "${GREEN} [TEST] COMPARISON OF EXTRACTED AND EXPECTED MODEL SUCCESSFUL"
  exit 0
fi
