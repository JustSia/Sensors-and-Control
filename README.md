# Sensors and Control
 Fetch Robot Pick and Place Simulation
 
 # Setup
 1. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and setup a catkin workspace using the tutorial [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 2. Download the required fetch robot packages using the tutorial on the fetch robot [ROS Melodic installation guide](https://docs.fetchrobotics.com/indigo_to_melodic.html)
 3. Git clone this respository into your ~/catkin/src folder by using the git clone command as listed below or downloading it manually
  > cd ~/catkin_ws/src
 > git clone https://github.com/JustSia/Sensors-and-Control.git
 4. Run the cmake command and the following launch files in order to begin the simulation
 > cd ~/catkin_ws

 > catkin_make
 
 > source devel/setup.bash
 
 Setup up the simulated environment
 > roslaunch fetch_gazebo environment.launch 
 
 Running the simulation
 > roslaunch fetch_gazebo simulation.launch  
 
