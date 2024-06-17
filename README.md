<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width=30% height=30%>

From new parkour:
<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/parkour.png" width=30% height=30%>

From added RVIZ
<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/rviz.png" width=30% height=30%>

# BFMC Simulator Project

The project contains the entire Gazebo simulator. 
- It can also be used to get a better overview of how the competition is environment looks like
- It can be used in to develop the vehicle state machine
- It can be used to simulate the path planning
- It can be used to set-up a configuration procedure for the real track
- Not suggested for image processing
- Try not to fall in the "continuous simulator developing" trap

From KOU-Mekatronom team:
- It has robot_localization package, you can fuse the gps and IMU data easily.
- Robot_localization package config path is ```src/example/config/ekf_localization.yaml```
- Added urdf and lidar.sdf 
- It has laser_scan now topic name is ```/automobile/scan``` for bostacle_detection.
- Added TF2 package the tf tree visualization ```frames.pdf``` 


Tips on how to install and work on it, can be found in the 


## The documentation is available in details here:
[Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/simulator.html)
