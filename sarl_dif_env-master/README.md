# SARL* Tutorial

This repository is to help understanding how sarl* works and for gazebo simulation or real-world using sarl* with Turtlebot3. You can find original code at [here](https://github.com/LeeKeyu/sarl_star).

## 0. Contents   
#### 1. Build & Install   
#### 2. Training   
#### 3. Test the SARL* in gazebo simulation   
#### 4. Test the SARL* with Real Turtlebot3   
#### 5. Change the velocity limitation of Turtlebot3   
#### 6. Reference 

## 1. Build & Install

  1. Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
  2. Make you own workspace and clone this repository.
  
    mkdir -p ~/sarl_ws/src
    cd ~/sarl_ws/
    catkin_make
    source devel/setup.bash
    cd src
    git clone https://github.com/nabihandres/sarl_dif_env.git
  
  3. Install dependencies.

    sudo apt install libbullet-dev
    sudo apt install libsdl-image1.2-dev
    sudo apt install libsdl-dev
    sudo apt install ros-melodic-bfl ros-melodic-tf2-sensor-msgs
    pip install empy
    pip install configparser
    sudo apt install cmake
  
  4. Install [Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2).

    cd /sarl_ws/src/sarl_dif_env/Python-RVO2/
    pip install Cython
    python setup.py build
    python setup.py install or sudo python setup.py install 

  5. Install [CrowdNav](https://github.com/vita-epfl/CrowdNav).

    cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav 
    pip install -e .
    
    Note: if the installation is killed at 99% while 'torch' is being installed, reboot and try again with below command.

    pip install -e . --no-cache-dir
    
  6. Install Dependent ROS Packages [TurtleBot3-Robotis](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
    
    sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
    ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
    ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
    ros-melodic-rosserial-server ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
    ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-compressed-image-transport ros-melodic-rqt* \
    ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers	 
  
  7. Finally build the workspace.

    cd ~/sarl_ws/
    catkin_make
    source devel/setup.bash
    
    Note: if you have problem with **#include <people_msgs/PeoplePrediction.h>**
    clone the following package and replace de people inside documents with your current people document except wu_ros_tool folder

 
    https://github.com/marinaKollmitz/people.git
    

## 2. Training

  The training parameters can be found in "env.config", "policy.config", "train.config" under "~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/".


  1. Change simulation environmnets and parameters required for training, refering to 'SARL*_Intenship.md'.


  2. Train the robot and Make Value Network Model. 
 ``` 
    cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/

    python train.py --policy sarl
 ```

  3. Visualize a test case. 
 ```
    cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav

    python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 0
    
    sudo python3 test.py --policy cadrl --model_dir data/output_beta_cadrl_circular_10 --phase test --visualize --test_case 10
    
    sudo python3 test.py --policy lstm_rl --model_dir data/output1_LSTM_onlydz_square_invisible_ieee --phase test --visualize --test_case 1

 ```
   
**Note**: test_case is not the episode number. It is just the seed number.  
    ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/test.py  

   ```python
    if args.visualize:
        ob = env.reset(args.phase, args.test_case)
   ```

   * circle_crossing_old  

https://user-images.githubusercontent.com/83470394/161479285-8bab274d-2b87-4185-80af-44183e2693e9.mp4


  4. Plot training curve. If you want
     1. define the names of the models you want to plot,
     2. success rate plot, add "--plot_sr" flag, 
     3. collision rate plot, add "--plot_cr" flag, 
     4. episode time plot, add "--plot_time" flag.
     5. compare different training outcomes.

   4-1.  
   ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/utils/plot.py

   ```python
    # define the names of the models you want to plot
    # models = ['LSTM-RL', 'SARL', 'OM-SARL']
    models = ['output_new2_0.4_f1', 'output_new2_0.8_f1', 'output_new3_0.7_f1']
    max_episodes = 20000
   ```
   
   4-2.  
   
    cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav
    
    python utils/plot.py data/output/output.log
    # If you want to chage the output path
    python utils/plot.py path/to/your/output.log
    # Add suceess rate plot
    python utils/plot.py data/output/output.log --plot_sr
    # Add suceess rate plot, collision rate plot.
    python utils/plot.py data/output/output.log --plot_sr --plot_cr
    # Add success rate plot, collision rate plot, time rate plot.
    python utils/plot.py data/output/output.log --plot_sr --plot_cr --plot_time

   4-3.

    # If you want to compare different training outcomes, enter the path of the data you want to compare at the same time.
    python utils/plot.py data/output_new2_0.4_f1/output.log data/output_new2_0.8_f1/output.log data/output_new3_0.7_f1/output.log --plot_sr --plot_cr

**Note**: When comparing graphs of the same kind, all graphs must have the same range of x-axis and y-axis. Put plt.ylim().

   ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/utils/plot.py  

   ```python
        # plot sr
	...
            ax1.legend(ax1_legends)
            ax1.set_xlabel('Episodes')
            ax1.set_ylabel('Success Rate')
            ax1.set_title('Success rate')
            plt.grid(True)
            plt.ylim([0, 1.0]) 
   ```

## 3. Test the SARL* in gazebo simulation

### 3-1. Gazebo World Mapping

  1. Add following command in "~/.bashrc" to export "TURTLEBOT3_MODEL" environment.

    export TURTLEBOT3_MODEL=burger

  2. Launch gazebo simulation.

    roslaunch sarl_star_ros turtlebot3_gazebo.launch

  (optional) Initial position can be changed by "x_pos", "y_pos", "z_pos" arguments. If you want to change gazebo world, change "world_file" argument to world file path. Now, gazebo world is from "sarl_world.world".

 ```xml
  <!--turtlebot3_gazebo.launch-->
  <launch>
    <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
    <arg name="x_pos" default="0.0"/>
    <arg name="y_pos" default="0.0"/>
    <arg name="z_pos" default="0.0"/>
    <!--Change the world file you want-->
    <arg name="world_file" default="$(find sarl_star_ros)/worlds/sarl_world.world" 
      doc="[$(find sarl_star_ros)/worlds/columns_world.world, 
            $(find turtlebot3_gazebo)/worlds/turtlebot3_world.world, 
            $(find turtlebot3_gazebo)/worlds/empty_world.world]
            $(find turtlebot3_gazebo)/worlds/sarl_world.world]" />

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="world_name" value="$(arg world_file)"/>
      <arg name="paused" value="false"/>
      <arg name="use_sim_time" value="true"/>
      <arg name="gui" value="true"/>
      <arg name="headless" value="false"/>
      <arg name="debug" value="false"/>
    </include>  

    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model turtlebot3_burger -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />

  </launch>
 ```

  3. Mapping with gmapping and move Turtlebot3 with teleop key.

    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch # w, a, s, d

  4. After finishing the mapping, save the map using map_saver. Then you can find the .yaml and .pgm files.

    rosrun map_server map_saver -f <PATH_TO_SAVE>
    #Ex) 
    rosrun map_server map_saver -f ~/sarl_map

   * Demo video

https://user-images.githubusercontent.com/87967303/153540906-24bfa86f-c93c-4870-b4e7-bd477e3178eb.mp4

  5. Move map files to the "/map" directory under the "sarl_star_ros" package, modify the path of the pgm in the yaml file.  

     ```
     image: ./[pgm file name].pgm
     resolution: 0.050000
     origin: [-10.000000, -10.000000, 0.000000]
     negate: 0
     occupied_thresh: 0.65
     free_thresh: 0.196
     ```

  6. Make a new launch file for the new world under "sarl_star_ros/launch/", changing the "world_file" and "map_file" arguments to your path.

 ```xml
  <launch>

    <!-- Define laser type-->
    <arg name="laser_type" default="hokuyo" />

    <!-- laser driver -->
    <!-- <include file="$(find turtlebot_navigation)/laser/driver/$(arg laser_type)_laser.launch" /> -->

    <!-- Gazebo simulation -->
    <arg name="world_file" default="$(find sarl_star_ros)/worlds/sarl_world.world"/>

    <include file="$(find sarl_star_ros)/launch/turtlebot3_gazebo.launch">
      <arg name="world_file" value = "$(arg world_file)"/>
    </include>

    <!-- Publish turtlebot3 robot state-->
    <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch"/>

    <!-- laser filter (optional) -->
    <node pkg="laser_filters" type="scan_to_scan_filter_chain"
        name="laser_filter">
      <rosparam command="load" file="$(find laser_filters)/laserscan_filter.yaml" />
    </node>

    <!-- Map server -->
    <arg name="map_file" default="$(find sarl_star_ros)/map/sarl_map.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

    <!-- AMCL -->
    <arg name="custom_amcl_launch_file" default="$(find sarl_star_ros)/launch/includes/amcl/amcl.launch.xml"/>

    <arg name="initial_pose_x" default="0"/>
    <arg name="initial_pose_y" default="0"/>
    <arg name="initial_pose_a" default="0"/>

    <include file="$(arg custom_amcl_launch_file)">
      <arg name="initial_pose_x" value="$(arg initial_pose_x)"/>
      <arg name="initial_pose_y" value="$(arg initial_pose_y)"/>
      <arg name="initial_pose_a" value="$(arg initial_pose_a)"/>
    </include>

    <!-- People Detector
    <include file="$(find people_velocity_tracker)/launch/tracked_detector.launch" />-->

    <!-- Obstacle Detector-->
    <include file="$(find obstacle_detector)/launch/nodes.launch" />

    <!-- Move base -->
    <arg name="custom_param_file" default="$(find sarl_star_ros)/param/$(arg laser_type)_costmap_params.yaml"/>
    <include file="$(find sarl_star_ros)/launch/includes/move_base/move_base.launch.xml">
      <arg name="custom_param_file" value="$(arg custom_param_file)"/>
    </include>

    <!-- SARL_star Planner -->
    <node pkg="sarl_star_ros" type="sarl_star_node.py" name="sarl_star_node" output="screen" />

    <!-- rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find sarl_star_ros)/rviz/sarl_crowd_nav.rviz"/>


  </launch>

 ```



### 3-2. Test Trained Model with Gazebo Turtlebot3

  1. You can set parameters by refering to 'SARL*_Intership.md'.

  2. Add following command in "~/.bashrc" to export "TURTLEBOT3_MODEL" environment.

    export TURTLEBOT3_MODEL=burger

  3. Launch gazebo simulation.

    Without obstacles: roslaunch sarl_star_ros turtlebot3_sarl_star_world.launch

    with obstacles: roslaunch deepexpress_gazebo simulation.launch

**Note**: About "simulation.launch", refer to "*SARL*_COOP.md".

  4. To set goal, select "2D Nav Goal" in upper panel of rviz and click the position in the map.

**Note**: When robot achieves the goal, sarl_star_node is teminated. If you want to plan another goal, set another goal and run following command. **Also, error may happen, but it will be run by running the command again and again.** (I don't know why but tf listner has some problems)

    $ rosrun sarl_star_ros sarl_star_node.py


## 4. Test the SARL* with Real Turtlebot3

### 4-1. Turtlebot3 Bringup

  1. Add "ROS_MASTER_URI", "ROS_HOSTNAME" in your PC's "~/.bashrc" as following.

    export ROS_MASTER_URI=http://<YOUR_PC_IP>:11311
    export ROS_HOSTNAME=<YOUR_PC_IP>
    Ex)
    export ROS_MASTER_URI=http://192.168.0.210:11311
    export ROS_HOSTNAME=192.168.0.210
    
  And save.
  
    source ~/.bashrc

  
  Then, run roscore in your PC.

    roscore
  
  2. Turn on Turtlebot3 and make sure your PC and Turtlebot3 are connected to same wifi.

  3. SSH into Turtlebot3 and enter password, "raspberry"

    ssh pi@<Turtlebot3 IP>
    Ex)
    ssh pi@192.168.0.226 
    # password: "raspberry"

  4. Open "~/.bashrc" file, edit "ROS_MASTER_URI" and "ROS_HOSTNAME" and ctrl-s -> ctrl-x to save and exit.

    nano ~/.bashrc
    # edit "export ROS_MASTER_URI=http://<YOUR_PC_IP>:11311"
    # edit "export ROS_HOSTNAME=<Turtlebot3_IP>"
    # Ex)  "export ROS_MASTER_URI=http://192.168.0.210:11311"
    # Ex)  "export ROS_HOSTNAME=192.168.0.226"
    source ~/.bashrc

  5. Launch Turtlebot3 bringup.

    roslaunch turtlebot3_bringup turtlebot3_robot.launch  # General Turtlebot 3
    roslaunch turtlebot3_bringup turtlebot3_urg.launch  # Turtlebot3 using hokuyo lidar
  

  6. Then, you can see the following topics.

    rostopic list

    /cmd_vel
    /diagnostics
    /firmware_version
    /imu
    /joint_states
    /laser_status
    /left_pwm
    /magnetic_field
    /motor_power
    /odom
    /opmode_state
    /present_pwm_L_state
    /present_pwm_R_state
    /present_vel_L_state
    /present_vel_R_state
    /reset
    /right_pwm
    /rosout
    /rosout_agg
    /scan
    /sensor_state
    /sound
    /tf
    /tf_static
    /version_info
    /wheel_control

### 4-2. Real World Mapping

  1. Mapping with gmapping and move Turtlebot3 with teleop key. Make sure that executing following commands in your PC (remote PC)

    roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch # w, a, s, d

  2. After finishing the mapping, save the map using map_saver.

    rosrun map_server map_saver -f ~/<MAP_NAME>
    
https://user-images.githubusercontent.com/83470394/182323588-9f849300-4ddd-4a69-b8c8-344acfc50a70.mp4
   

### 4-3. Test Trained Model with Real Turtlebot3

  1. Move map files to the "sarl_star_ros/map" directory under the "sarl_star_ros" package, modify the path of the pgm in the yaml file.
  
     ```
     image: ./[pgm file name].pgm
     resolution: 0.050000
     origin: [-10.000000, -10.000000, 0.000000]
     negate: 0
     occupied_thresh: 0.65
     free_thresh: 0.196
     ```

  2. Change the "map_file" argument to your map path under "sarl_star_ros/launch/sarl_star_navigation.launch"

 ```xml
  <launch>

    <!-- Define laser type-->
    <arg name="laser_type" default="hokuyo" />

    <!-- laser driver -->
    <!-- <include file="$(find turtlebot_navigation)/laser/driver/$(arg laser_type)_laser.launch" /> -->

    <!-- laser filter (optional) -->
    <node pkg="laser_filters" type="scan_to_scan_filter_chain"
        name="laser_filter">
      <rosparam command="load" file="$(find laser_filters)/laserscan_filter.yaml" />
    </node>

    <!-- Map server -->
    <arg name="map_file" default="$(find sarl_star_ros)/map/lab2.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

    <!-- AMCL -->
    <arg name="custom_amcl_launch_file" default="$(find sarl_star_ros)/launch/includes/amcl/amcl.launch.xml"/>

    <arg name="initial_pose_x" default="0.0"/>
    <arg name="initial_pose_y" default="0.0"/>
    <arg name="initial_pose_a" default="0.0"/>

    <include file="$(arg custom_amcl_launch_file)">
      <arg name="initial_pose_x" value="$(arg initial_pose_x)"/>
      <arg name="initial_pose_y" value="$(arg initial_pose_y)"/>
      <arg name="initial_pose_a" value="$(arg initial_pose_a)"/>
    </include>

    <!-- People Detector
    <include file="$(find people_velocity_tracker)/launch/tracked_detector.launch" /> -->

    <!-- Obstacle Detector-->
    <include file="$(find obstacle_detector)/launch/nodes.launch" />

    <!-- Move base -->
    <arg name="custom_param_file" default="$(find sarl_star_ros)/param/$(arg laser_type)_costmap_params.yaml"/>
    <include file="$(find sarl_star_ros)/launch/includes/move_base/move_base.launch.xml">
      <arg name="custom_param_file" value="$(arg custom_param_file)"/>
    </include>

    <!-- SARL_star Planner -->
    <node pkg="sarl_star_ros" type="sarl_star_node.py" name="sarl_star_node" output="screen" />

    <!-- rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find sarl_star_ros)/rviz/sarl_crowd_nav.rviz"/>


  </launch>
 ```
 
  3. After finishing chaging map argument, restart Turtlebot3 bringup.

    roslaunch turtlebot3_bringup turtlebot3_robot.launch  # General Turtlebot 3
    roslaunch turtlebot3_bringup turtlebot3_urg.launch  # Turtlebot3 using hokuyo lidar

  4. You can set parameters by refering to 'SARL*_Internship.md'.

  5. Add following command in "~/.bashrc" to export "TURTLEBOT3_MODEL" environment.

    export TURTLEBOT3_MODEL=burger

  6. Launch SARL* and related packages (map server, amcl, obstacle detector, move base, sarl_star_node). 

    roslaunch sarl_star_ros sarl_star_navigation.launch

  7. Set the goal in rviz.

**Note**: When robot achieve the goal, sarl_star_node is teminated. If you want to plan another goal, set another goal and run following command. **Also, error may happen, but it will be run by running the command again and again.** (I don't know why but tf listner has some problems)


    $ rosrun sarl_star_ros sarl_star_node.py

https://user-images.githubusercontent.com/83470394/182323497-365884d1-60f5-4b7f-8c77-4530f53d668c.mp4

## 5. Change the velocity limitation of Turtlebot3

  1. Check the limitation RPM of the dynamixel.

  2. Go to the link below and do 1 through 9 of **1.3. Setup DYNAMIXELs for TurtleBot3**.

    https://emanual.robotis.com/docs/en/platform/turtlebot3/faq/#faq

  3. Please find proper core example from **examples > tutlebot3 > turtlebot3_burger or turtlebot3_waffle > turtlebot3_core > turtlebot3_burger.h or turtlebot3_waffle.h**.

  4. Change the **MAX_LINEAR_VELOCITY** from the code below.


   ```c
    #ifndef TURTLEBOT3_BURGER_H_
    #define TURTLEBOT3_BURGER_H_
  
    #define NAME                             "Burger"
  
    #define WHEEL_RADIUS                     0.033           // meter
    #define WHEEL_SEPARATION                 0.160           // meter (BURGER : 0.160, WAFFLE : 0.287)
    #define TURNING_RADIUS                   0.080           // meter (BURGER : 0.080, WAFFLE : 0.1435)
    #define ROBOT_RADIUS                     0.105           // meter (BURGER : 0.105, WAFFLE : 0.220)
    #define ENCODER_MIN                      -2147483648     // raw
    #define ENCODER_MAX                      2147483648      // raw
  
    #define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
    #define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s
  
    #define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
    #define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 
  
    #endif  //TURTLEBOT3_BURGER_H_
   ```

  5. Upload the example to OpenCR.
  

## 6. Reference

SARL* : https://github.com/LeeKeyu/sarl_star

CrowdNav : https://github.com/vita-epfl/CrowdNav

Turtlebot3 : https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

URG LIDAR : https://github.com/ros-drivers/urg_node

Obstacle detector : https://github.com/tysik/obstacle_detector


