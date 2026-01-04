#!/bin/bash

# 1. SETUP & CLEANUP
# We use 'sudo' for cleanup just in case, but ignore errors if password is asked
echo ">>> Cleaning up..."
pkill -f gazebo
pkill -f rosmaster
sleep 2

# 2. SOURCE ROS (Crucial!)
# We try both common locations
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source /root/catkin_ws/devel/setup.bash
elif [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source $HOME/catkin_ws/devel/setup.bash
else
    echo "WARNING: Could not find setup.bash automatically."
fi

# 3. DOWNLOAD TO HOME DIR (Fixes Permission Denied)
# Instead of /root, we use $HOME (~)
ASSET_DIR="$HOME/gazebo_assets"

if [ ! -d "$ASSET_DIR" ]; then
    echo ">>> Downloading GitHub Office Assets to $ASSET_DIR..."
    git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git "$ASSET_DIR"
fi

# Set Gazebo Paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$ASSET_DIR/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$ASSET_DIR/worlds

# 4. OPTIMIZE SHADOWS
# We fix the file at the new location
WORLD_FILE="$ASSET_DIR/worlds/office_env_large.world"
if [ -f "$WORLD_FILE" ]; then
    sed -i 's/<shadows>1<\/shadows>/<shadows>0<\/shadows>/g' "$WORLD_FILE"
    sed -i 's/<shadows>true<\/shadows>/<shadows>false<\/shadows>/g' "$WORLD_FILE"
else
    echo " ERROR: World file not found at $WORLD_FILE"
    exit 1
fi

# 5. LAUNCH OFFICE WORLD
echo ">>> Launching Office World..."
roslaunch gazebo_ros empty_world.launch world_name:="$WORLD_FILE" paused:=true gui:=true verbose:=true &
GAZEBO_PID=$!

echo ">>> Waiting 20 seconds for world to load..."
sleep 20

# 6. SPAWN ROBOT
echo ">>> Finding Robot File..."
# We search in the current user's workspace
ROBOT_FILE=$(find $HOME/catkin_ws/src -name "limo_mycobot.xacro" 2>/dev/null | head -n 1)

# Fallback to root if not found in home
if [ -z "$ROBOT_FILE" ]; then
   ROBOT_FILE=$(find /root/catkin_ws/src -name "limo_mycobot.xacro" 2>/dev/null | head -n 1)
fi

if [ -z "$ROBOT_FILE" ]; then
    echo " ERROR: Could not find limo_mycobot.xacro! Check your workspace."
else
    echo ">>> Spawning Robot at X=2.0, Y=2.0..."
    rosparam set robot_description "$(xacro $ROBOT_FILE)"
    rosrun gazebo_ros spawn_model -urdf -param robot_description -model limo_cobot -x 2.0 -y 2.0 -z 0.2
fi

# 7. START CONTROLLERS
echo ">>> Starting Controllers..."
# Try to find the control file dynamically
CONTROL_CONFIG=$(find $HOME/catkin_ws/src -name "limo_cobot_control.yaml" 2>/dev/null | head -n 1)
if [ -n "$CONTROL_CONFIG" ]; then
    rosparam load "$CONTROL_CONFIG"
fi

roslaunch limo_cobot_gazebo limo_cobot_control.launch &

echo "DONE! Click PLAY (▶) in Gazebo."
wait $GAZEBO_PID
