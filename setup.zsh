#!/usr/bin/env zsh

# Modify this for your environment

if [[ -z "${ARDUPILOT_GAZEBO}" ]]; then
  export ARDUPILOT_GAZEBO="$HOME/workspace/thirdparty/ardupilot_gazebo"
fi

if [[ -z "${ARDUPILOT_HOME}" ]]; then
  export ARDUPILOT_HOME="$HOME/workspace/thirdparty/ardupilot"
fi

if [[ -z "${COLCON_WS}" ]]; then
  export COLCON_WS="$HOME/workspace/colcon-ws-acfr"
fi

if [[ -z "${COLCON_WS_LOCAL}" ]]; then
  export COLCON_WS_LOCAL="$HOME/workspace/colcon-ws-acfr"
fi


# Add results of ArduSub build
export PATH=${ARDUPILOT_HOME}/build/sitl/bin:$PATH

# Add results of colcon build
source ${COLCON_WS}/install/setup.zsh
source ${COLCON_WS_LOCAL}/install/setup.zsh

# Add ardupilot_gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=${ARDUPILOT_GAZEBO}/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

# Add bluerov2_gz models and worlds
export GZ_SIM_RESOURCE_PATH=${COLCON_WS}/src/bluerov2_gz/models:${COLCON_WS}/src/bluerov2_gz/worlds:$GZ_SIM_RESOURCE_PATH

# Add orca4 models and worlds
export GZ_SIM_RESOURCE_PATH=${COLCON_WS_LOCAL}/src/orca4/orca_description/models:${COLCON_WS_LOCAL}/src/orca4/orca_description/worlds:$GZ_SIM_RESOURCE_PATH

# Build ros_gz on the humble branch for Gazebo Harmonic
export GZ_VERSION=harmonic
