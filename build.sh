colcon build --packages-skip haptiquad_ros2 haptiquad_contacts
source install/setup.bash
colcon build --cmake-force-configure --packages-select haptiquad_ros2 haptiquad_contacts
source install/setup.bash