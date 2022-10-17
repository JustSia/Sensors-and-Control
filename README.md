# Sensors and Control
 Fetch Robot Pick and Place Simulation using Ubuntu 18.04 and ROS Melodic
 
 Contributors: Justin Sia, Felix Watson and Tun Tun Lin
 
 ## Project Description
 <!DOCTYPE html>
<html>
<body>
  
  <iframe src="https://drive.google.com/file/d/1k5hlrz_heq6OpywA9cQgUaULV4gMZsIM/preview" width="640" height="480" allow="autoplay"></iframe>
  <iframe allowfullscreen="allowfullscreen" src="your_page_url/preview" ></iframe>

</body>
</html>
 <iframe src="https://drive.google.com/file/d/1k5hlrz_heq6OpywA9cQgUaULV4gMZsIM/preview" width="640" height="480" allow="autoplay"></iframe>
 
 The goal of this project is to demonstrate the use of visual servoing in the fetch robot in order to grasp objects. In order to demonstrate this, a simple task was setup in which the robot was required to move differently shaped objects to a designated goal within a dining kitchen environment. 
 
 ## Installation and Setup Guide
 1. [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and setup a catkin workspace using the tutorial [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 2. Download the required fetch robot packages using the tutorial on the fetch robot [ROS Melodic installation guide](https://docs.fetchrobotics.com/indigo_to_melodic.html)
 3. Git clone this respository into your ~/catkin/src folder or downloading it manually
 * Clone the repo
 ```sh
 cd ~/catkin_ws/src
 git clone https://github.com/JustSia/Sensors-and-Control.git
 ```
 4. Run the cmake command and the following launch files in order to begin the simulation
 * Build the workspace
 ```sh
 cd ~/catkin_ws
 catkin_make
 ```
 * Run the simulation
 ```sh
 source devel/setup.bash
 roslaunch fetch_gazebo environment.launch 
 roslaunch fetch_gazebo simulation.launch  
 ```
