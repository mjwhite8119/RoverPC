# RoverPC ROS2 

This is the PC side of the Rover project.  The git PC directory source of this project resides at `~/dev_ws/src/RoverPC` and gets mounted into a docker container `duasynv/ros:humble-ros-base-l4t-r34.1.1`

### Compile and Run
Within the docker container:

    cd dev_ws/src
    colon build --symlink-install