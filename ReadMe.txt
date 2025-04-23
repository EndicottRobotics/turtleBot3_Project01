#terminal 0:------ clone project ---------------------------------
~/ros2_ws/src/turtlebot3_ws/src
git clone https://github.com/EndicottRobotics/turtlebot3_Project01

best to start code. in the following directory
~/ros2_ws/src/turtlebot3_ws/src/turtlebot3_Project01

HOWEVER! 
compile in the following directory
~/ros_ws 


#-------------- To run ----------------------------------

# terminal 1: (robot simulation)
source ~/.bashrc
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# terminal 2: (tf mapping)
source ~/.bashrc
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

# terminal 3: (navigation node)
source ~/.bashrc
ros2 run turtlebot3_Project01 navigate_to_tf_target_node

# terminal 4:(set target node)
source ~/.bashrc
ros2 run turtlebot3_Project01 input_target_tf_node
