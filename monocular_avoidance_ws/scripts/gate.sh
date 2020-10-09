#!/bin/zsh

if [[ $# -eq 2 ]]; then
    echo "Location of gate recieved: ($1, $2)"
else
    1=2
    2=0
    echo "Requires two arguments - Assuming default location of gate ($1, $2)"
fi

# You will first need to give the FOLDER (GATEMODELS) listed in "gatefile" permission to be changed by the current user

# This is the gate models file
gatefile="/opt/parrot-sphinx/usr/share/sphinx/models/GateModels/model.sdf"

# replace the position of the gate
sed -i "6s/.*/      <pose>$1 $2 -1.5 0 0 1.570796<\/pose>/" $gatefile

sphinx ~/Desktop/MixedRealityTesting/monocular_avoidance_ws/src/drone_controller/worlds/empty_gate.world /opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone::name=first::stolen_interface=::pose="0 0 0.2 0 0 0"::with_front_cam=true