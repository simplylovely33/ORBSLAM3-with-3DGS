# ORB-SLAM3 with 3D Gaussian Splatting
This is a project that combines ORB-SLAM3 and 3D Guassian Splatting for real-time camera pose estimation and map reconstruction

# Prerequisites
Camera selection is Intel Realsense D435, we have implemented the project with **ROS noetic** and **NVIDIA GeForce RTX 3090Ti** on **Ubuntu 20.04**.

# Update Log
2026.6.11 

We use the Intel Realsense D435 as the capturing device, and use the `ROS Camera Calibration` package to calibrate the camera.
```
sudo apt-get install ros-noetic-usb-cam  ### Usb cam package installation
sudo apt install ros-noetic-camera-calibration  ### Calibration package installation
roslaunch usb_cam usb_cam-test.launch  ### Open the camera from /usb_cam/image_raw
rosrun camera_calibration cameracalibrator.py --size H*V --square S image:=/usb_cam/image_raw  ### Calibrate the camera with corresponding setting
```
`H,V` in `--size` indicate the number of horizontal and vertical inner corners respectively. `S` in `--square` indicate the size of each square on the checkboard, in meters. After you click the SAVE button in the display window, the calibration data is saved in '/tmp/calibrationdata.tar.gz'

2026.6.12

We use the `roslaunch` to read the rgb and depth images from the camera.
```
roslaunch realsense2_camera rs_rgbd.launch  ### Launch file to open the camera reading node.
```
But there is still an error occur when open the current frame and ORB-SLAM3 Map Viewer window. Try to use the `gdb` command to debug.
```
gdb /Examples/ROS/ORB_SLAM3/RGBD  ### Getting into the debug mode
(gdb) run Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM3/MyD435.yaml /camera/rgb/image_raw:=/camera/color/image_raw /camera/depth_registered/image_raw:=/camera/aligned_depth_to_color/image_raw  ### Configuration Setting
```
`LEFT:=RIGHT`， left indicate the name of subscribe topic and right express the input topic. 

2026.6.13

We change the chain of think thread to begin with ORB-SLAM2 instead of beginning with ORB-SLAM3 (because there is clear open-source tutorial)
```
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/Examples/ROS/ORB_SLAM2  ### Configure ROS environment variables.
roslaunch realsense2_camera rs_rgbd.launch   ### Launch file to open image read node.
rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/RGB-D/RealSenseD435.yaml /camera/rgb/image_raw:=/camera/color/image_raw /camera/depth_registered/image_raw:=/camera/aligned_depth_to_color/image_raw  ### Start ORB-SLAM2(RGBD) command
```

2026.6.16

Read the 2025 CVPR Best Paper [VGGT: Visual Geometry Grounded Transformer](https://arxiv.org/pdf/2503.11651), which is a severely impressive work.
Reproduce and local model deployment,

2026.6.17

Prepare in-house dataset for VGGT application.
Input 55 images with a resolution of 3072*2048 to reach the memory limit.
```
python demo_gradio.py  ### Launch the Gradio Web Interface by local machine
>>  ERROR OCCUR: TypeError: argument of type 'bool' is not iterable  >>  pip install pydantic==2.10.6
```

Use an SSH tunnel on the user end to connect to the server port：
```
ssh -L PORT:localhost:PORT user@your.server.ip  # Access server port
```
**DIFFUCULT**:

Shells or overlapping parts appear on the point cloud surface reconstructed from the ring sequence due to VGGT's lack of visible loop closure optimization.
```
python demo_colmap.py --scene_dir DATA_PATH --use_ba  #  Use Bundle Adjustment to eliminate the error
```
Performs well in continuous image camera movements, but fails in ring photography studios where camera settings are far apart.

2026.6.18

New computer environment configuration setting with **NVIDIA GeForce RTX 4060** on **Ubuntu 24.04**

Network deployment [Clash-verge](https://github.com/clash-verge-rev/clash-verge-rev/releases/download/v2.0.3/Clash.Verge_2.0.3_amd64.deb)
```
sudo dpkg -i Clash.Verge_2.0.3_amd64.deb
```

Anaconda installation [Tsinghua Mirror](https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/?C=M&O=D) with required version (Example:[Anaconda3-2023.09-0-Linux-x86_64.sh](https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/Anaconda3-2023.09-0-Linux-x86_64.sh))
```
bash Anaconda3-2023.09-0-Linux-x86_64.sh
sudo gedit ~/.bashrc  #  Edit the environment
export PATH=~/anaconda3/bin:$PATH  #  Add environment variable
source ~/.bashrc  #  Save the environment
conda -V  #  Check anaconda version
```
Configure the Tsinghua download mirror
```
conda config
sudo gedit ~/.condarc
```












