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
          Subscriber { name 'power_state' message 'sensor_msgs.BatteryState'}}
        Parameters {
          Parameter { name 'string_test' type String value 'test'},
          Parameter { name 'bool_tets' type Boolean },
          Parameter { name 'int_test' type Integer },
          Parameter { name 'struc_test/last_element/hola' type Integer },
          Parameter { name 'double_test' type Double },
          Parameter { name 'struc_test/first_element' type Integer }}}
}}}"

declare -a InterfacesArray=("ServiceServer { name 'setBool' service 'std_srvs.SetBool'}"
"ServiceClient { name 'init' service 'std_srvs.Trigger'}"
"Publisher { name 'scan' message 'sensor_msgs.LaserScan'}"
"Subscriber { name 'power_state' message 'sensor_msgs.BatteryState'}"
"Parameter { name 'string_test' type String value 'test'}"
"Parameter { name 'bool_tets' type Boolean }"
"Parameter { name 'int_test' type Integer }"
"Parameter { name 'struc_test/last_element/hola' type Integer }"
"Parameter { name 'double_test' type Double }"
"Parameter { name 'struc_test/first_element' type Integer }")

bash /haros_runner.sh test_pkg test_node node .  /home/extractor/ws "https://github.com/ipa-nhg/test_ros_code_extractor -b ${1}"

resulted_model=$(cat  /home/extractor/ws/test_node.ros | sed -E 's,\\t|\\r|\\n,,g')

for interface in "${InterfacesArray[@]}"
do
  if ! echo "$resulted_model" | grep -q "$interface" 
  then
    echo -e "${RED} [TEST] EXTRACTION FAILS"
    echo -e "${RED} EXTRACTED AND EXPECTED MODEL DON'T MATCH"
    echo "The following interface couldn't be found:"
    echo "~~~"
    echo "$interface"
    echo "~~~"
    exit 1
  fi
done

echo -e "${GREEN} [TEST] BUILD SUCCESSFUL"
echo -e "${GREEN} [TEST] EXTRACTION SUCCESSFUL"
echo -e "${GREEN} [TEST] COMPARISON OF EXTRACTED AND EXPECTED MODEL SUCCESSFUL"
exit 0
