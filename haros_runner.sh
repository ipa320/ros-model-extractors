#!/bin/bash

# Arguments:
#   1: Package name
#   2: Node name or launch file name or '--all' to analyse all the available nodes 
#   3: Type of the request: either 'launch' or 'node'
#   4: Path to the folder where the resulting model files should be stored
#   5: Path to the ROS workspace 
#   (optional) from 6: Http address links of the Git repositories (to indicate the branch put the name of the repository between quotes and add the suffix -b *banch_name*, for example "https://github.com/ipa320/ros-model-extractors b main" 
# Returns:
#   (None)

# scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

number_of_args=$#
number_of_repos=$((number_of_args-5))

for repo in "${@:6:$number_of_repos}"
do
  cd "${5}"/src
  git clone $repo
done

cd "${5}"
model_repo=$(echo "${6}" | sed 's/ .*//')

if [ -d "${5}/devel" ]; then
  source devel/setup.bash
else
  source install/setup.bash
fi


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
    source ${5}/install/setup.bash
    colcon list > /tmp/colcon_list.txt
    path_to_src_code=$(cat /tmp/colcon_list.txt |  grep "^$1" | awk '{ print $2}')
    if [ -z "$path_to_src_code" ]; then
      echo "** ERROR: Package ${1} not found in the workspace **"
      exit
    fi
    path_to_src_code="${5}/$path_to_src_code"
  else
    echo "ROS version not supported"
    exit
  fi
else
  echo "ROS installation not found"
fi

echo ""

#tree ${5}

echo "## Init HAROS ##"
mkdir -p "${4}"
haros init

echo ""
echo "## Call the HAROS plugin to extract the ros-models ##"
if [ -n $ROS_PYTHON_VERSION ]
then
  if [[ $ROS_PYTHON_VERSION == "2" ]]
  then
    if [ "${2}" = "--all" ]
    then
      python /home/divya/Documents/Master_Thesis/ros_model_extractot_fork/ros-model-extractors/ros_model_extractor.py --package "$1" --"${3}" --model-path "${4}" --ws "${5}" --repo $model_repo -a>> extractor.log
    else
      python /home/divya/Documents/Master_Thesis/ros_model_extractot_fork/ros-model-extractors/ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}" --repo $model_repo>> extractor.log
    fi
    #cat extractor.log
  elif [[ $ROS_PYTHON_VERSION == "3" ]]
  then
    if [ "${2}" = "--all" ]
    then
      python3 /home/divya/Documents/Master_Thesis/ros_model_extractot_fork/ros-model-extractors/ros_model_extractor.py --package "$1" --"${3}" --model-path "${4}" --ws "${5}" --path-to-src "$path_to_src_code" --repo $model_repo -a >> ${4}/extractor.log
    else
      python3 /home/divya/Documents/Master_Thesis/ros_model_extractot_fork/ros-model-extractors/ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}" --path-to-src "$path_to_src_code" --repo $model_repo>> ${4}/extractor.log
    fi
    #cat extractor.log 
  else
    echo "Python version not supported"
    exit
  fi
else
  echo "Python setup not found"
fi


echo "~~~~~~~~~~~"
echo "Extraction finished. See the following report:"
cat ${4}/extractor.log
echo "~~~~~~~~~~~"

echo "###########"
for generated_model in "${4}"/*.ros
do
echo "~~~~~~~~~~~"
echo "Print of the model: $generated_model:"
echo "~~~~~~~~~~~"
cat $generated_model
echo ""
echo "~~~~~~~~~~~"
echo "###########"
done

## Clean and finish
# rm -rf ${5}/src/*
# rm -rf ${4}/*
