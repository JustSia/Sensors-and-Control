<a name="readme-top"></a>
# Sensors and Control
### **Fetch Robot Pick and Place Simulation using Ubuntu 18.04 and ROS Melodic**
![Screenshot 2022-10-18 111105](https://user-images.githubusercontent.com/114462972/196306266-876cbc99-95ba-4eff-8eec-b6b92747ec01.png)
 

| Contributors  | Description   | Percentage Contribution |
| ------------- | ------------- | ----------------------- | 
| Justin Sia    | Perception and Visual Servoing  | 33%                     |
| Felix Watson  | Object and Environment Generation  | 33%                     |
| Tun Tun Lin  | Arm and Base Movement Kinematics | 33%                     |
 
 ## Project Description

https://user-images.githubusercontent.com/114462972/197919760-2bbb8018-af49-4930-9697-f5fc90627e82.mp4


The goal of this project is to demonstrate the use of visual servoing in the fetch robot in order to grasp objects. In order to demonstrate this, a simple task was setup in which the robot was required to move differently shaped objects to a designated goal within a dining kitchen environment. 
 
 ## Installation and Setup Guide
 1. [Install](http://wiki.ros.org/melodic/Installation/Ubuntu) ROS Melodic and setup a catkin workspace using the tutorial [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
 2. Download the required fetch robot packages using the tutorial on the fetch robot [ROS Melodic installation guide](https://docs.fetchrobotics.com/indigo_to_melodic.html)
 3. Git clone this respository into your ~/catkin/src folder or downloading it manually
 * Clone the repo
 ```sh
 cd ~/catkin_ws/src
 git clone https://github.com/JustSia/Sensors-and-Control.git
 ```
 4. Make sure the code is executable
 ```sh
 cd ~/catkin_ws/src/Sensors-and-Controlfetch_gazebo/fetch_gazebo_demo/scripts
 sudo chmod +x simulation.py
 ```
 ```sh
 cd ~/catkin_ws/src/Sensors-and-Controlfetch_gazebo/fetch_gazebo/scripts
 sudo chmod +x prepare_simulated_robot.py
 sudo chmod +x prepare_simulated_robot_pick_place.py
 ```
 6. Run the cmake command and the following launch files in order to begin the simulation
 * Build the workspace
 ```sh
 cd ~/catkin_ws
 catkin_make
 ```
 * Run the simulation
 ```sh
 source ~/catkin_ws/devel/setup.bash
 roslaunch fetch_gazebo environment.launch 
 ```
 * Open a new terminal and run the following to start the simulation
 ```sh
 source ~/catkin_ws/devel/setup.bash
 roslaunch fetch_gazebo_demo simulation.launch  
 ```
 ## Troubleshooting
 #### 1. Error: "RLException: [filename] is neither a launch file in package [filename] nor is [filename] a launch file name."
 Make sure the source is set to the right place every time a new terminal or tab is open
 ```
 source ~/catkin_ws/devel/setup.bash
 ```
 
 #### 2. Error: "CMake Error: Could not find a package configuration file provided by "name""
 Install missing packages, replace name with the missing package name, use "-" for spaces between the name.
 ```
 sudo apt install ros-melodic-name
 ```
 
 #### 3. Error: "resource not found: name"
 Install missing packages, replace name with the missing package name, use "-" for spaces between the name.
 ```
 sudo apt install ros-melodic-name
 ```
 

<p align="right">(<a href="#readme-top">back to top</a>)</p>
