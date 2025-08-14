# NSL-3130CV ROS2
--- NSL-3130CV ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
2. Build NSL-3130CV demo
```
$ cd NSL3130_driver
$ colcon build --packages-select roboscan_nsl3130
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 run roboscan_nsl3130 roboscan_publish_node
$ ros2 launch roboscan_nsl3130 camera.Launch.py
```

# NSL-3130AA View

  ![ROS2](https://user-images.githubusercontent.com/106071093/218378867-6792ac7b-4b2f-4227-9fa3-ef833f0fc784.png)

# Phase wrapping Avoidance and Correction

  ![phase2](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/4a134327-213f-4558-9fa7-47de543866c9)
  ![roll-over-gif2](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/b3a04a07-6822-4076-8dc5-69eed91bc935)

# Average FPS per Image type

  ![fps](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/532178f4-23ac-4eee-ae8d-a4f8fb03b747)


# Set parameters
```
$ rqt
 (reconfigure)
```

![Screenshot from 2023-02-22 13-00-21 (copy)](https://user-images.githubusercontent.com/106071093/220520356-3d16736f-902c-4d9e-858d-a6ed8ddf87aa.png)


```

cvShow : Image Viewer on/off
channel : 0 ~ 15

hdr_mode 0 : HDR off
hdr_mode 1 : Spatial HDR
hdr_mode 2 : Temperal HDR

imageType 0 : STOP MODE 
imageType 1 : DISTANCE_MODE 
imageType 2 : DISTANCE_AMPLITUDE_MODE
imageType 3 : RGB
imageType 4 : RGB_DISTANCE_MODE
imageType 5 : RGB_DISTANCE_AMPLITUDE_MODE

int0 , int1, int2 = 0 ~ 2000
intGr = 0 ~ 40000

modIndex 0 : 12MHz
modIndex 1 : 24MHz
modIndex 2 : 6MHz
modIndex 3 : 3MHz


transformAngle : angle (rviz-based y-axis rotation)
```

 



 
 
 
