# PX4 UAV uses Mavros in Gazebo-classic simulation

## Install PX4
> ### 1. Download PX4 source code
> ```bash
> git clone https://github.com/PX4/PX4-Autopilot.git --recursive
> sudo apt update && apt install -y wget curl lsb-release gnupg apt-utils
> ```
>
> ### 2. Run the ubuntu.sh with no arguments to install everything
> ```bash
> bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
> ```
>
> ### 3. Compile PX4 SITL and Gazebo
> ```bash
> cd PX4-Autopilot
> git submodule update --init --recursive
> DONT_RUN=1 make px4_sitl_default gazebo-classic
> ```
---
## Install catkin_tools
> ```bash
> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
> wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
> sudo apt-get update
> sudo apt-get install python3-catkin-tools
>  ```
---
## Install MAVROS
> ### 1. Use version of ROS noetic on Ubuntu 20.04
> ```bash
> sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
> ```
>
> ### 2. Install GeographicLib datasets
> ```bash
> wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
> sudo bash ./install_geographiclib_datasets.sh
> ```
> 
---
## MAVROS Offboard control example (Python)
> ### 1. Build work space and creating the ROS package
> ```bash
> mkdir -p ~/catkin_ws/src
> cd ~/catkin_ws/
> catkin init
> cd ~/catkin_ws/src
> catkin_create_pkg offboard_py rospy
> catkin build
> echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> roscd  # Should cd into ~/catkin_ws/devel
>
> roscd offboard_py
> mkdir scripts
> cd scripts
> ```
>
> ### 2. Using sample code
> ```bash
> touch offb_node.py
> chmod +x offb_node.py
> ```
>> open offb_node.py file and paste the following code:
>> ```python
>> #! /usr/bin/env python3
>>
>> import rospy
>> from geometry_msgs.msg import PoseStamped
>> from mavros_msgs.msg import State
>> from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
>>
>> current_state = State()
>>
>> def state_cb(msg):
>>     global current_state
>>     current_state = msg
>>
>>
>> if __name__ == "__main__":
>>     rospy.init_node("offb_node_py")
>>
>>     state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
>>
>>     local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
>>
>>     rospy.wait_for_service("/mavros/cmd/arming")
>>     arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
>>
>>     rospy.wait_for_service("/mavros/set_mode")
>>     set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
>>
>>
>>     # Setpoint publishing MUST be faster than 2Hz
>>     rate = rospy.Rate(20)
>>
>>     # Wait for Flight Controller connection
>>     while(not rospy.is_shutdown() and not current_state.connected):
>>         rate.sleep()
>>
>>     pose = PoseStamped()
>>
>>     pose.pose.position.x = 0
>>     pose.pose.position.y = 0
>>     pose.pose.position.z = 2
>>
>>     # Send a few setpoints before starting
>>     for i in range(100):
>>         if(rospy.is_shutdown()):
>>             break
>>
>>         local_pos_pub.publish(pose)
>>         rate.sleep()
>>
>>     offb_set_mode = SetModeRequest()
>>     offb_set_mode.custom_mode = 'OFFBOARD'
>>
>>     arm_cmd = CommandBoolRequest()
>>     arm_cmd.value = True
>>
>>     last_req = rospy.Time.now()
>>
>>     while(not rospy.is_shutdown()):
>>         if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
>>             if(set_mode_client.call(offb_set_mode).mode_sent == True):
>>                 rospy.loginfo("OFFBOARD enabled")
>>
>>             last_req = rospy.Time.now()
>>         else:
>>             if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
>>                 if(arming_client.call(arm_cmd).success == True):
>>                     rospy.loginfo("Vehicle armed")
>>
>>                 last_req = rospy.Time.now()
>>
>>         local_pos_pub.publish(pose)
>>
>>         rate.sleep()
>> ```
>
> ### 3. Creating the ROS launch file
> ```bash
> roscd offboard_py
> mkdir launch
> cd launch
> touch start_offb.launch
> ```
>> open start_offb.launch and paste the following code:
>> ```XML
>> <?xml version="1.0"?>
>> <launch>
>>  <!-- Include the MAVROS node with SITL and Gazebo -->
>>	 <include file="$(find px4)/launch/mavros_posix_sitl.launch">
>>	 </include>
>>
>>	 <!-- Our node to control the drone -->
>>	 <node pkg="offboard_py" type="offb_node.py" name="offb_node_py" required="true" output="screen" />
>> </launch>
>>```
>>
>
> ### 4. Little change in ~/.bashrc file
>> add these lines at the end of .bashrc file:
>>
>> ```bash
>> source ~/catkin_ws/devel/setup.bash
>> source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
>> export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
>> export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
>> export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
>> ```
>>
>> run the following command after editing .bashrc
>> ```bash
>> source ~/.bashrc
>> ```
>
> ### 5. Launching your script
> ```bash
> roslaunch offboard_py start_offb.launch
> ```
>
> <details>
>  <summary>參考資料</summary>
>    1. <a href="https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html">Ubuntu Development Environment</a><br>
>    2. <a href="https://docs.px4.io/main/en/ros/mavros_installation.html#ros-noetic-(ubuntu-22.04)">ROS (1) with MAVROS Installation Guide</a><br>
>    3. <a href="https://catkin-tools.readthedocs.io/en/latest/installing.html">Installing catkin_tools</a><br>
>    4. <a href="https://docs.px4.io/main/en/ros/mavros_offboard_python.html">MAVROS Offboard control example (Python)</a><br>
>    5. <a href="https://github.com/PX4/PX4-Autopilot/issues/14762">ERROR: cannot launch node of type [px4/px4]</a><br>
>    6. <a href="https://discuss.px4.io/t/unable-to-run-mavros-and-px4-with-rlexception-error/32925">Unable to run MAVROS and PX4 with RLException error</a><br>
>    7. <a href="https://hackmd.io/@zjewp/mavros">PX4, MAVROS, and Gazebo Installation</a><br>
> </details>
