# killall -9 roscore
# killall -9 rosmaster
# roscore &
# # rviz -d src/flocking/rviz/flocking.rviz &
# roslaunch ego_planner my_rviz.launch &

source /opt/ros/noetic/setup.bash && source devel/setup.sh &&  roslaunch ego_planner my_rviz.launch