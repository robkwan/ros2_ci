#! /bin/bash 
#set -euo pipefail

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

#prints each command and its arguments as they are executed for debugging
set -x

# Exit immediately if a command exits with a non-zero status
set -e

#xhost +local:root &

# Print current working directory
echo "Current working directory: $(pwd)"
echo "Container PATH: $PATH"

#Launch fastbot gazebo
echo "$(date +'[%Y-%m-%d %T]') Starting fastbot_gazebo..."
ros2 launch fastbot_gazebo one_fastbot_room.launch.py &
BG_PID=$!

# Wait for services to initialize
echo "[$(date '+%Y-%m-%d %T')] Waiting for services to initialize..."
sleep 2  # Give some time for Gazebo to start
until ros2 node list 2>/dev/null >/dev/null; do sleep 1; done
until ros2 node list | grep -q '/gazebo'; do sleep 1; done

#Launch fastbot action server
echo "$(date +'[%Y-%m-%d %T]') Starting fastbot action server..."
ros2 run fastbot_waypoints fastbot_action_server &
AS_PID=$!

# Wait for the action server to be ready
echo "[$(date '+%Y-%m-%d %T')] Waiting for waypoint action server to start..."
sleep 30
until ros2 action list | grep -q '/fastbot_as'; do sleep 1; done

#Launch ros tests 
echo "$(date +'[%Y-%m-%d %T]') Starting ros tests..."
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
TEST_RESULT=$?
set -e

echo " * RESULT: $( [ $TEST_RESULT -eq 0 ] && echo SUCCESS || echo FAILURE )"

# Shutdown background processes
echo "Stopping background processes..."
kill "$BG_PID" 2>/dev/null || true
kill "$AS_PID" 2>/dev/null || true


# Keep the script running
#echo "[$(date '+%Y-%m-%d %T')] All components are running. Press Ctrl+C to exit."
#while true; do
#    sleep 1  # Prevents busy-waiting
#done

# --- Exit with the real test result ---
exit $TEST_RESULT
