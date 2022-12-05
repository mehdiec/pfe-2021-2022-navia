#!/bin/bash 
#gnome-terminal --tab  --title="set TF" -- bash -c "ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map world"


gnome-terminal --tab  --title="manageimage" -- bash -c "colcon build --packages-select manageimage --allow-overriding manageimage --symlink-install;
. install/setup.bash ;
cd manageimage/launch;
ros2 launch manageimage manageimage.launch.py;"

gnome-terminal --tab  --title="camerasetup" -- bash -c "colcon build --packages-select camerasetup --allow-overriding visualisation --symlink-install;
. install/setup.bash ;
cd camerasetup/launch;
ros2 launch camerasetup camera.launch.py;"

gnome-terminal --tab  --title="visualisation" -- bash -c "colcon build --packages-select visualisation --allow-overriding visualisation --symlink-install;
. install/setup.bash ;
cd visualisation/launch;
ros2 launch visualisation visualisation.launch.py;"

#gnome-terminal --tab  --title="manageimage_pub" -- bash -c ". install/setup.bash ;
#ros2 run manageimage publisher_test;"

gnome-terminal --tab  --title="translation" -- bash -c "colcon build --packages-select translation_broadcaster --allow-overriding translation_broadcaster --symlink-install;
. install/setup.bash ;
ros2 run translation_broadcaster broadcast;"

gnome-terminal --tab  --title="rviz" -- bash -c "ros2 run rviz2 rviz2"



#bash"
#colcon build --packages-select visualisation --allow-overriding visualisation --symlink-install
#. install/setup.bash
#cd visualisation/launch 
#ros2 launch visualisation visualisation.launch.py

#!/bin/bash 
#gnome-terminal -- command
#ls