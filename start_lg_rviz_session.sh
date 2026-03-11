#!/bin/bash
# cd /home/user/lightning-lm
# ros0sh='source /opt/ros/humble/setup.bash && source install/setup.bash'
cd /home/user/lightinglm_ws
ros2sh='source /opt/robot/scripts/setup_ros2.sh && source install/setup.bash'
ros0sh='source /opt/ros/foxy/setup.bash && source install/setup.bash'

# Start a new tmux session named 'lg'
tmux new-session -d -s lg

tmux new-window -t lg:0 -n 'lightning'
tmux send-keys -t lg:0 "$ros2sh && ros2 run lightning run_slam_online --config src/lightning-lm-deep-robotics/config/default_deep_roboticsslam.yaml" C-m

# Tab 1: Static transform publisher
tmux new-window -t lg:1 -n 'navstate'
tmux send-keys -t lg:1 "$ros2sh && rviz2 -d src/lightning-lm-deep-robotics/config/showbodypc.rviz" C-m

# Tab 2: Check node info, save map service
tmux new-window -t lg:2 -n 'node_info'
tmux send-keys -t lg:2 "$ros2sh && sleep 5 && ros2 topic echo /lightning/nav_state" C-m
# tmux send-keys -t lg:2 "ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'office4f'}"" C-m

tmux new-window -t lg:3 -n 'node_info'
tmux send-keys -t lg:3 "$ros2sh && exec bash" C-m

# Attach to the tmux session
tmux attach-session -t lg
