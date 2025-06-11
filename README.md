# ORB-SLAM3 with 3D Gaussian Splatting
This is a project that combines ORB-SLAM3 and 3D Guassian Splatting for real-time camera pose estimation and map reconstruction

# Prerequisites
Camera selection is Intel Realsense D435, we have implemented the project with **ROS noetic** on **Ubuntu 20.04**.

# Update Log
2026.6.11 

We use the Intel Realsense D435 as the capturing device, and use the `ROS Camera Calibration` package to calibrate the camera.
```
sudo apt-get install ros-noetic-usb-cam    ### Usb cam package installation
sudo apt install ros-noetic-camera-calibration    ### Calibration package installation
roslaunch usb_cam usb_cam-test.launch    ### Open the camera from /usb_cam/image_raw
rosrun camera_calibration cameracalibrator.py --size H*V --square S image:=/usb_cam/image_raw
```
`H,V` in `--size` indicate the number of horizontal and vertical inner corners respectively. `S` in `--square` indicate the size of each square on the checkboard, in meters.
