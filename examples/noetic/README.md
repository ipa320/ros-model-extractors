### Command lines to auto-generate the examples of this folder

```
docker run -it haros_noetic:latest /haros_runner.sh hokuyo_node hokuyo_node node . /root/ws https://github.com/ros-drivers/hokuyo_node

docker run -it haros_noetic:latest /haros_runner.sh fake_localization fake_localization node . /root/ws https://github.com/ros-planning/navigation

docker run -it haros_noetic:latest /haros_runner.sh cob_mimic mimic node . /root/ws https://github.com/ipa320/cob_driver

docker run -it haros_noetic:latest /haros_runner.sh cob_sick_s300 cob_sick_s300 node . /root/ws https://github.com/ipa320/cob_driver

docker run -it haros_noetic:latest /haros_runner.sh cob_light cob_light node . /root/ws https://github.com/ipa320/cob_driver

```
