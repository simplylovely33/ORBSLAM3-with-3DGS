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

New computer environment configuration setting with **NVIDIA GeForce RTX 4060** and **5090** on **Ubuntu 24.04**

Network deployment [Clash-verge](https://github.com/clash-verge-rev/clash-verge-rev/releases/download/v2.0.3/Clash.Verge_2.0.3_amd64.deb)
```
sudo dpkg -i Clash.Verge_2.0.3_amd64.deb
```

**Anaconda** installation on [Tsinghua Mirror](https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/?C=M&O=D) with required version (Example:[Anaconda3-2023.09-0-Linux-x86_64.sh](https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/Anaconda3-2023.09-0-Linux-x86_64.sh))
```
bash Anaconda3-2023.09-0-Linux-x86_64.sh
sudo gedit ~/.bashrc  #  Edit the environment
export PATH=~/anaconda3/bin:$PATH  #  Add environment variable
source ~/.bashrc  #  Save the environment
conda -V  #  Check anaconda version
```
Configure the Tsinghua download mirror by context in [.condarc](https://github.com/simplylovely33/ORBSLAM3-with-3DGS/blob/main/.condarc)
```
conda config
sudo gedit ~/.condarc  #  Copy the context into the file
conda config --show-sources
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple  #  Configure the pip download source
```

**Pycharm** installation(Community) [Download](https://download.jetbrains.com/python/pycharm-community-2025.1.2.tar.gz?_gl=1*1xp5ksj*_gcl_au*MTQzMDYwMjcxOS4xNzUwMjE2MDMx*FPAU*MTQzMDYwMjcxOS4xNzUwMjE2MDMx*_ga*MTU2MTIxOTQxNC4xNzUwMjE2MDMy*_ga_9J976DJZ68*czE3NTAyMzE0MjckbzIkZzEkdDE3NTAyMzE2MjgkajU2JGwwJGgw)
```
tar -zxvf pycharm-community-2025.1.2.tar.gz
rm pycharm-community-2025.1.2.tar.gz
cd pycharm-community-2025.1.2
./bin/pycharm.sh
```
Remember choose the `Tools`->`Create Desktop Entry...` to load the Pycharm Community shortcut

**Cmake** installation [Download](https://cmake.org/files/) (Example:[Cmake-3.26.0](https://cmake.org/files/v3.26/cmake-3.26.0.tar.gz))
```
tar -zxvf cmake-3.26.0.tar.gz
rm cmake-3.26.0.tar.gz && cd cmake-3.26.0
./bootstrap  #  Examination the cmake
(option) sudo apt-get install libssl-dev  #  if the OPENSSL ERROR occur
make -j  # Complie the cmake
sudo make install #  Install the software or tool
```

**Cudatoolkit** installation. The torch version must align with the cuda version in [3D Gaussain Splatting](https://arxiv.org/pdf/2308.04079), so we install the convenient version [Cuda12.1](https://developer.nvidia.com/cuda-12-1-1-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=runfile_local)
```
wget https://developer.download.nvidia.com/compute/cuda/12.1.1/local_installers/cuda_12.1.1_530.30.02_linux.run
sudo sh cuda_12.1.1_530.30.02_linux.run
```
If you have already install the driver, remember cancel the `driver` button.
```
echo 'export PATH=/usr/local/cuda/bin:/usr/local/cuda-12.1/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda-12.1/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
source ~/.bashrc
```

2026.6.19

SSH client **Termius** installation. Failed to download from the official website, using `snapd` to install the app.
```
sudo apt install snapd
sudo snap install termius-app
(option) sudo snap remove --purge termius-app  #  In case enter the wrong info, clear all configuration
```
You can deploy the different versions of cudatoolkit for the different demand. All your need is to configure the corresponding environment variable.
```
export PATH=/usr/local/cuda-xx.x/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-xx.x/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
`cuda-xx.x` indicate your installed cudatoolkit version.

ERROR occur when using `NVIDIA GeForce RTX 5090` capability sm_120 is not compatible with the current PyTorch installation.
```
pip uninstall -y torch torchvision torchaudio  #  Delete the exist torch
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128  #  Install the sm_120 support torch 
```


2026.6.20

**MeshLab** installation
```
sudo apt install meshlab  #  Most convenient command
```

VGGT on the masked dataset for the ring shot view are not satisfactory due to the fact that VGGT treats all pixels (including black) as valid pixels. Even after masking and `Filter Black Background`, the estimated camera references are not accurate.

What works is to do the calculation via `demo_colmap.py`
```
python demo_colmap.py --scene_dir YOUR_DATA --query_frame_num 24 --max_query_pts 1024 --shared_camera (option)--use_ba
```
But there is problem that the result from `--use_ba`(Bundle Adjustment) is worse than without `--use_ba` ???

Change to use the different angle view from the camera setting instead of the same row camera capture.












