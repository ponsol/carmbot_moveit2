# carmbot_moveit2

C++ ROS2 node for armbot_moveit2
This works only using a named pose. 

colcon build --mixin debug --packages-select carmbot_moveit2

ros2 run carmbot_moveit2 carmbot_moveit2 --ros-args --params-file params.yaml 

