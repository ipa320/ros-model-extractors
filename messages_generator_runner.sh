#!/bin/bash

# Arguments:
#   1: Package name
#   2: Path to the folder where the resulting model files should be stored
#   3: Path to the ROS workspace 
#   4: Http address links of the Git repositories
# Returns:
#   (None)

cd "${3}"
source devel/setup.bash
source install/setup.bash

repo_link="${4}"

cd "${3}"/src
if [ "$#" -ne 3 ]  &&  [ "${#repo_link}" -ne 3 ]; then
  git clone ${4}
  rosdep update
  cd "${3}"

  echo ""
  echo "## Install ROS pkgs dependencies ##"
  if [ -n $ROS_VERSION ]
  then
    if [[ $ROS_VERSION == "1" ]]
    then
      source devel/setup.bash
      rosdep install -y -i -r --from-path src
      catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
    elif [[ $ROS_VERSION == "2" ]]
    then
      source install/setup.bash
      rosdep install -y -i -r --from-path src
      colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    else
      echo "ROS version not supported"
      exit
    fi
  else
    echo "ROS installation not found"
  fi

  echo ""
fi



mkdir -p "${2}"
rm "${2}"/"${1}".ros 2> /dev/null
/bin/bash /generate_messages_model_helper.sh "${1}" >> "${2}"/"${1}".ros
echo "FINISH!"
rm -rf "${3}"/src/*
