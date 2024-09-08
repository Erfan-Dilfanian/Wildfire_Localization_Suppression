
# Autonomous Real-time Wildfire Detection, Global Localization and Suppression

PLease find the ROS workspace source code in `framework_code/M300_ws`

## Tasks before experimetnal test

- Charge batteries:
  battery of drones: big ones and small ones
  battery of their remote controllers: big ones and small ones
  RTK battery
  Battery boxes
- Make sure you have the correct weights for detection
- See whether geolocalization part of your code works with rosbag recorded file

## checklist

- iCrests
- wood
- lighter
- gasoline
- propane pit
- fire pits
- Colored water
- RTK
- DJI mini/Phantom/mavic for filming
- Arduino
- Tape
- Scissor
- Toolbox
- Monitor
- Keyboard
- laptop


## during test
**First test:** 
- RTK off
- Ground Truth GPS
- second launch file on to open the valve
- Lateral adjustment

**second test:** 
- RTK off
- Ground Truth GPS
- second launch file on
**- detection on**
**- fuzzy controller on**

**third test:**
- **RTK on**

**fourth test:**
-**test geopositioning**

**Commands:**
### m300_ws

#### Dependencies
```bash
sudo apt install ros-noetic-rosserial-msgs

pip3 install pyserial
```

fuzzylite

after installing fuzzylite:

export LD_LIBRARY_PATH=/home/qin/fuzzylite/build/bin:$LD_LIBRARY_PATH

note: please change the path to weights in the M300_ws/src/forest_fire_geopositioning/sctipts/fire_detection_v8.py

#### Usage

Place the M300 toward the north.

##### ssh into the iCrest 2.0

```bash
ssh - X erfan@192.168.1.26
```

##### main launch file
First, you need to source your `setup.bash`:
```bash
cd M300_ws
source devel/setup.bash
roslaunch dji_osdk_ros dji_vehicle_node.launch 

```


##### Detection launch file

There is is a second launch file that you need to launch. It consists of compress_video_node (in which we also have main_and_fpv_node), the geopositioing node, and one of the arduino nodes.
Open a new terminal.

```bash
roslaunch forest_fire_geopositioning fire_detection.launch 
```

you can also open this one:
```bash
rqt_image_view
```


##### To run the fire detection (on ground station laptop)
you need to run the following node in the stationay machine. (directly run themin the stationay PC not in the iCrest)
```bash
rosrun forest_fire_geopositioning fire_detection_v8.py
```

**note:** you need to git clone M300_Ws respository on the ground station laptop as well

##### To visualize the fire bounding boxes (optional)
rosrun forest_fire_geopositioning fire_spots_visualization

##### To run the fire localization and SLAM (on groundstation laptop)
To run the SLAM manually:
```bash
rosrun ORB_SLAM3 fire_localization /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Examples_old/Monocular/GoPro.yaml
```
note that this node is in the ground station PC


##### Fighting nodes
```bash
rosrun forest_fire_detection_system InMotionDropping_Flight_control_node
```

*note:* after each flight test, Ctrl+C this node and the second launch file, and replug the arduino power cable so that you make arduino ready for the next flight.

##### To individually test Retardant-releasing nodes
The first node is in the fire _geopositioning launch file, and the second one would be called form the code, so these are just for testing the arduino. 

Open a new terminal

First, you need to run the following node:
```bash
rosrun arduino_actuator serial_node.py /dev/ttyUSB0
```
Then, whenever you were ready, release the retardant with the following node:
```bash
rosrun arduino_actuator servo_pub.py
```



#### Dateset available
https://drive.google.com/file/d/1YPX3RgdjjUx_tRMU9sRcODnS2XP0VS_U/view?usp=sharing


#### create your own dataset
rosbag record -O m300_dataset /bounding_boxes/fire_spots /clock /dji_osdk_ros/gps_position /dji_osdk_ros/main_wide_RGB /position/camera_pose /position/fire_spots /position/fire_spots_GPS /position/real_scale /dji_osdk_ros/imu

#### there are some useful yaml files that you can use


##### Topics
```
/bounding_boxes/fire_spots
/clock
/dji_osdk_ros/gps_position
/dji_osdk_ros/main_wide_RGB
/position/camera_pose
/position/fire_spots
/position/fire_spots_GPS
/position/real_scale
```
