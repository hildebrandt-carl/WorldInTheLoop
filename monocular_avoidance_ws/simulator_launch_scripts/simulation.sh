#!/bin/zsh

sphinx ~/MixedReality/monocular_avoidance/catkin_ws/src/drone_controller/worlds/empty.world /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::name=first::stolen_interface=::pose="0 0 0.2 0 0 0"::with_front_cam=true /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::name=second::stolen_interface=::pose="7 0 0.2 0 0 0"::with_front_cam=false