#!/bin/bash
# cd /home/user/lightning-lm
# ros0sh='source /opt/ros/humble/setup.bash && source install/setup.bash'
cd /home/user/lightinglm_ws
ros2sh='source /opt/robot/scripts/setup_ros2.sh && source install/setup.bash'
ros0sh='source /opt/ros/foxy/setup.bash && source install/setup.bash'

# Start a new tmux session named 'loc'
tmux new-session -d -s loc

tmux new-window -t loc:0 -n 'lightning'
tmux send-keys -t loc:0 "$ros2sh && ros2 run lightning run_loc_online --config src/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml" C-m

# Tab 1: Static transform publisher
tmux new-window -t loc:1 -n 'navstate'
tmux send-keys -t loc:1 "$ros2sh && rviz2 -d src/lightning-lm-deep-robotics/config/showglobalmap.rviz" C-m

# Tab 2: Check node info, save map service
tmux new-window -t loc:2 -n 'node_info'
tmux send-keys -t loc:2 "$ros2sh && sleep 5 && ros2 topic echo /lightning/nav_state" C-m

tmux new-window -t loc:3 -n 'node_info'
tmux send-keys -t loc:3 "$ros2sh && exec bash" C-m
# tmux send-keys -t loc:2 "ros2 service call /lightning/save_path lightning/srv/SavePath "{file_path: 'data/traj.txt'}"" C-m

# Attach to the tmux session
tmux attach-session -t loc
