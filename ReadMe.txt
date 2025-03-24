



-------------------- Handy linux commands -------------------------

---change directory
cd 
---list files in directory
ls 
---search for some text in a file
grep -n TextToFind NameOf File
---copy a file
cp FileName CopyFileName 
---remove a file
rm FileName 
---move a file
mv FileName path/NewFileName  

--------------- how to start vscode for just this project -----------

cd ~/ros2_ws/src/project_03
code .

--------------- how to do a complile  ---------------------------------

------ for project_03  
------ ~/ros2_ws/src/project_03  (do it in this directory )
colcon build 

------ clean complile on all projects  (this take a while)
------ ~/ros2_ws  (do it in this directory )
colcon clean workspace
   answer (Y)
colcon build 

--------------- run setup.bash so executables can be found in new terminals
source ~ros2/_ws/install/setup.bash

--------------- how run run executables for this project-----------------
------ one exectable per terminal
ros2 run turtlesim turtlesim_node
ros2 run project_03 goto_target_node
ros2 run project_03 sequence_node
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot


------ this launch file start the goto_target_node and sequence_node in the same terminal
ros2 launch project_03 test.launch.py

--------------- handy ros commands to help debug -----------------
ros2 topic list
ros2 topic echo /turtle1/pose
ros2 topic echo /turtle1/cmd_vel
ros2 topic echo /turtle1/color_sensor

ros2 node list
ros2 pkg executables  


------ how to fix a messed up rqt_plot window
------ kill terminal running rqt_plot
cd ~/.config/ros.org/
rm rqt_gui.ini

