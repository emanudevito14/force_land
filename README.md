Before cloning this repository, you need to follow the README at this [link](https://github.com/emanudevito14/hexacopter_model) to use PX4_Autopilot with my hex_6 model. 
Furthermore, this repository must be used with the px4_msgs package [here](https://github.com/PX4/px4_msgs/tree/main), 
the QGroundControl application, and the DDS_run.sh file (which you can find [here](https://github.com/RoboticsLab2025/aerial_robotics)).

Clone this repository and px4_msgs into the ros2_ws/src directory
```
git clone https://github.com/emanudevito14/force_land.git
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs
git checkout release/1.16
```

Start QGroundControl application

Go to PX4-Autopilot folder
```
make px4_sitl gz_hex_6
```
Open new terminal and go to the folder where you put DDS_run.sh
```
. DDS_run.sh
```
Open another terminal and go in ros2_ws
```
colcon build
source install/setup.bash
ros2 run force_land force_land
```
You can verify the Force Land node's behavior by executing the takeoff command on QGroundControl 
and exceeding the threshold. If you regain control with the virtual joystick before it lands, 
you can also override the threshold, as shown in this [video](https://www.youtube.com/watch?v=cNRfQfLczzY)

Open new terminal
```
source install/setup.bash
ros2 run plotjuggler plotjuggler
```
In plotjuggler following this [video](https://www.youtube.com/watch?v=XoxQ2aJgwMs) to import metadata.yaml of the force_land/bags/force_land_bag folder. 
In the video, I use an absolute filter to plot the z-coordinate as −z for the ENU reference frame. However, it is better to use the Scale/Offset 
filter and set −1.0 as the value multiplier in the scale section.



