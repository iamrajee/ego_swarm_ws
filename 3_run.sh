source ~/swarm_ws/devel/setup.bash && roslaunch darknet_ros my_yolov3.launch config_file:=yolo-drone image:=stereo_camera/left/image_rect_color
#source devel/setup.bash && rosrun ego_planner publish_goals_for_all.py
# source devel/setup.bash && rosrun ego_planner publish_goals.py