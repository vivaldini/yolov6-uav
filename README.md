# Yolo v6 Object Detection from UAV Images 

This repository contains a ROS noetic package for YOLOv6 to recognize objects from UAV and provide their positions.
Based on ROS Package for Yolov6 developed by @lukazso
This package works together with MRS System, it is a easy to simulate UAV.


## Build Status    
| Component 	       |  20.04              | 
| ------------------- | ------------------- |
| RMA 	             | ![image](https://user-images.githubusercontent.com/74054598/149457205-fd48db89-0658-4511-af36-bcd8662562da.png)|

Features

   - UAV
   - Bluefox or Real-sense D-435
   - Odometry

## Dependencies and Requirements

### Step 1 - Instal MRS System

Follow the instructions [here](https://github.com/ctu-mrs/mrs_uav_system#installation) (on Installation topic) to install MRS System or use the commands:

```bash 
cd /tmp
echo '
GIT_PATH=~/git
mkdir -p $GIT_PATH
cd $GIT_PATH
sudo apt-get -y install git
git clone https://github.com/ctu-mrs/mrs_uav_system
cd mrs_uav_system
git checkout master
git pull
./install.sh -g $GIT_PATH
source ~/.bashrc' > clone.sh && source clone.sh
```
### Step 2 - Environment

Clone this repository into your catkin workspace, and looking for the setup and install dependencies:

```bash
cd ~/Your_worspace/src 
git clone https://github.com/JorgelHenri/ocean_trash_world
```


### Step 3 - Yolo

Following ROS packages are required:

-  [vision_msgs](http://wiki.ros.org/vision_msgs)

```bash
sudo apt-get install ros-noetic-vision-msgs
roscd vision_msgs 
```

-  [geometry_msgs](http://wiki.ros.org/geometry_msgs)

```bash
sudo apt-get install ros-noetic-geometry-msgs
roscd geometry_msgs  
```

Clone this repository into your catkin workspace:

```bash
cd ~/Your_worspace/src
git clone https://github.com/vivaldini/Yolo 
```

The Python requirements are listed in the requirements.txt. You can simply install them as

```bash
cd ~/Your_worspace/src/yolov6-uav
pip install -r requirements.txt 
```

Build the package 

```bash
cd ~/Your_worspace/src/yolov6-uav
catkin build yolov6-uav
```

### Using the simulator and Testing the Yolo

- Parameters configuration
 
In the file ~/Your_worspace/ocean_trash_world/offshore_uav_pack/start/session.yml, line 21 change the world environment and if necessary the camera
"ballontrash.launch" to "trash.launch"

In the file ~/Your_worspace/yolov6-uav/src/detect.py line 150, change it for your worspace and choose the weigths for your trainning models. For the garbage set as yolo_v6_simulated.pt

Before you launch the yolov6.launch node, adjust the camera and YOLOv6 weights topics in the launch file according to the sensors and topics that you are using. 

- Run the simulator 

This scenario opens a sea area containing some garbage and spawn a UAV.

```bash
cd ~/Your_worspace/src/ocean_trash_world/offshore_uav_pack/start
$ ./start.sh
```
In the Gazebo to start the simulation press the button ***Play


- Starting the Yolo

Copy the files of trainning available in this [drive](https://drive.google.com/drive/u/0/folders/1ulBtT66721qDCoDw9gpyJTilHD3LmQww) inside the folder "/yolov6-uav/launch/yolo_weigths"

```bash
roslaunch yolov6-uav yolov6.launch
```

Each time a new image is received it is then fed into YOLOv6.

***Notes
- The detections are published using the [vision_msgs/Detection2DArray](http://docs.ros.org/en/api/vision_msgs/html/msg/Detection2DArray.html) message type.
- The detections will be published under `/yolov6/out_topic`.
- If you set the `visualize` parameter to `true`, the detections will be drawn into 
  the image, which is then published under `/yolov6/yolov6/visualization`.


- Move the UAV

In the terminal where you executed the environment, go to the "Status tab" and press 

-- "h" to appear the command control of the UAV, and 

-- "Shift + R" to operate the UAV manually.

![Screenshot from 2023-03-30 14-16-59](https://user-images.githubusercontent.com/74054598/228833216-ce64ae7a-a875-41e6-83d8-03bf29a018fb.png)

***Note: Move the UAV to view the garbage.

- Visualization

```bash
rqt_image_view
topic /yolo6/yolov6/visualization
```
![Screenshot from 2023-03-30 13-53-28](https://user-images.githubusercontent.com/74054598/228828567-a44b462c-7bf4-4cdc-a4c1-f6e04534b927.png)



### Extra 1: Ways to stop the simulation

Press “CTRL + a” and after “k” in the prompt.

If something remains open:

```bash
alias killg='killall gzclient && killall gzserver && killall rosmaster'
killall px4
tmux kill-server
```

## Support

For support send email vivaldini@ufscar.br and dpsoler@usp.br 

