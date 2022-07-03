# RoverPC ROS2 

This is the PC side of the Rover project.  The git PC directory source of this project resides at `~/dev_ws/src/RoverPC` and gets mounted into a docker container.

## Setup on ROS PC

Start the container in detached mode:

    xhost +local:root
    ./start_navigation

Connect to container and source the environment:

    ./attach_navigation
    source /opt/ros/foxy/setup.bash

Test with:

    ros2 run demo_nodes_py listener

## Install ROS2 on Jetson
Make sure that docker is installed first and then use the docker image [jetbot_ros](github.com/dusty-nv/jetbot_ros)


    cd ~
    git clone https://github.com/dusty-nv/jetbot_ros
    cd jetbot_ros
    docker/run.sh

This will download the docker image the first time.

Create a workspace directory:

    mkdir dev_ws/src

Edit the script `~/jetbot_ros/docker/run.sh `that starts the container, and add the following line:

    -v $HOME/dev_ws:/dev_ws \

Stop and restart the container by exiting the container and then:

    cd ~/jetbot_ros
    docker/run.sh

There will be a directory `/dev_ws` where you can create your packages and they will persist across container restarts.    

Test:

    ros2 run demo_nodes_py talker
    ros2 run demo_nodes_py listener
    
Launch a new terminal session into the container:

    sudo docker exec -it jetbot_ros /bin/bash      

Build a single package:

    colcon build --packages-select rover_teleop --symlink-install

To launch an empty world in Gazebo:

    ros2 launch gazebo_ros gazebo.launch.py

To launch the Jetbo model in Gazebo:

    ros2 launch jetbot_ros gazebo_world.launch.py



### Clone Repository and Compile

To clone this repository and use it in the docker ROS2 container.  At the command line outside of the container:

    cd ~/dev_ws/src
    git clone https://github.com/mjwhite8119/RoverPC.git

This will show up within the docker container.  From within the docker container:

    cd dev_ws
    colon build --symlink-install

This will create the directories `build, install`, and `log`.  Note that these directories should be at the same level as the `src` directory.  
    
### Launch Rviz

First source the built environment:

    source install/setup.bash
