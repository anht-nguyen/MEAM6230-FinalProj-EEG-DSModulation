#!/usr/bin/env bash
# This script launches the full DS-based robot demo in a tmux session

# Adjust these paths if your workspace or project folder differ
CATKIN_WS="/catkin_ws"
PROJECT="MEAM6230-FinalProj-EEG-DSModulation"
SCRIPT_DIR="$CATKIN_WS/src/movewithhead/script"
SESSION="ds_demo"

# 1) Start a new tmux session
tmux new-session -d -s $SESSION -n setup

# 2) Window 0: launch Gazebo + MoveIt with DS support
tmux send-keys -t $SESSION:0 \
  "cd $CATKIN_WS && source devel/setup.bash && roslaunch movewithhead full_robot_arm_sim_with_ds.launch" C-m

# Wait for 10 seconds to allow Gazebo and MoveIt to initialize
sleep 10

# 3) Window 1: dualPosition node
tmux new-window -t $SESSION:1 -n dualPosition
tmux send-keys -t $SESSION:1 \
  "cd $CATKIN_WS && source devel/setup.bash && sudo chmod +x $SCRIPT_DIR/dualPosition.py && rosrun movewithhead script/dualPosition.py" C-m

# 4) Window 2: (optional) modulation input publisher
tmux new-window -t $SESSION:2 -n modInput
tmux send-keys -t $SESSION:2 \
  "cd $CATKIN_WS && source devel/setup.bash && sudo chmod +x $CATKIN_WS/src/attention_bridge/scripts/zmq_to_ros.py && rosrun attention_bridge scripts/zmq_to_ros.py" C-m

# 5) Window 3: plot modulation signal
tmux new-window -t $SESSION:3 -n rqt_plot
tmux send-keys -t $SESSION:3 \
  "cd $CATKIN_WS && source devel/setup.bash && rosrun rqt_plot rqt_plot /attention/global" C-m

sleep 10

# 6) Window 4: DS modulation executor
tmux new-window -t $SESSION:4 -n dsExecutor
tmux send-keys -t $SESSION:4 \
  "cd $CATKIN_WS && source devel/setup.bash && mosquitto -d && sudo chmod +x $SCRIPT_DIR/moveit_ds_modulation_executor.py && rosrun movewithhead script/moveit_ds_modulation_executor.py" C-m

# Wait for 10 seconds to allow Gazebo and MoveIt to initialize
sleep 10

# 7) Window 5: run demo action (e.g. waving)
tmux new-window -t $SESSION:5 -n runDemo
tmux send-keys -t $SESSION:5 \
  "cd $CATKIN_WS && source devel/setup.bash && sudo chmod +x $SCRIPT_DIR/run_a_demo.py && python3 $SCRIPT_DIR/run_a_demo.py 1" C-m

# 8) Attach to the tmux session
tmux attach-session -t $SESSION
