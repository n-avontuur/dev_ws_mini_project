cd ~/dev_ws_mini_project/ros-ws
colcon build
source install/setup.bash
ros2 run mini_project objectdetection_reciever_node
