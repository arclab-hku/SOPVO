#  sopvo
## 

### 1-Demo Video

[![](http://img.youtube.com/vi/VfmFdpIEhFM/0.jpg)](http://www.youtube.com/watch?v=VfmFdpIEhFM "")

### 2-Relevent Publication:

Paper Link: TBD

Project link: TBD

Abstruct:

This paper presents a novel outlier rejection approach for feature-based visual odometry. The proposed approach is based on an empirical observation that shows that some 2D-3D correspondences with very low reprojection error can cause a high error in the pose estimation. This work exploits such observation for odometry when a pair of stereo cameras are available. In fact, we argue that explicit pose error measure is desired over that of the implicit reprojection --whenever the former is possible-- to classify correspondences into inliers vs. outliers, for robust long term odometry. To explicitly measure the plausible pose error, we derive bounds on the individual pose parameters, with the help of the known orientation of stereo cameras. In this process, we formulate our bounds using the sum-of-square polynomials, which allow us to test whether a given correspondence satisfies any solution within the expected bounds. If the correspondence does not satisfy bounds for any parameter, it is considered to be an outlier. We implemented and tested the proposed method for UAV indoor navigation. The experiments from both benchmark and UAV onboard tests indicate that the inlier group refined by the proposed method significantly improves odometry estimation, compared to the traditional outlier rejection methods. In fact, the proposed method performs as accurately as IMU-aided methods in the state-of-the-art.

### 3-Support Camera/Dataset:
EuRoC MAV Dataset, Intel Realsense T265, KITTI(ros melodic)
### 4-Build The Project
We have tested in the following environment:

Ubuntu 16.04 + ROS Kinetic (recommend)

Ubuntu 18.04 + ROS melodic (parameters in config file need to be refined)

4.1 Clone the repository to the catkin work space eg. /catkin_ws/src

4.2 Install sopvo:
````
./install_sopvo_XXXX.sh
````
4.3 Compile
````
cd ~/catkin_ws
catkin_make
````
### 5-Verification
5.1 EuRoC MAV Dataset
Download the dataset(say MH_05_difficult) into the bag folder:
````
roscd sopvo/bag/
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_05_difficult/MH_05_difficult.bag
````
Edit the corresponding bag name in sopvo_euroc.launch file:
````
<node pkg="rosbag" type="play" name="rosbag" args="$(find sopvo)/bag/MH_05_difficult.bag"/>
````
run the following launch files:
````
roslaunch sopvo sopvo_euroc.launch
````

5.2 Intel Realsense T265 (live version):
Make sure you have installed realsense-ros properly. To run the live demo:
````
roslaunch sopvo sopvo_t265_live.launch
````

5.3 KITTI:
Please download the odometry dataset(gray image), then use kitti_img2rosbag_tool to convert the sequense into rosbag.
There are different image resolutions of the KITTI datasets, please modify the config files for the testing. 
In folder ./config we provide two examples: kitti_1226.yaml and kitti_1241.yaml. 

Demo video of KITTI tests:

[![](http://img.youtube.com/vi/0eGTsfJpJl4/0.jpg)](http://www.youtube.com/watch?v=0eGTsfJpJl4 "")

5.4 Other datasets:
Please wait for update.

### Maintainer:
Ran Duan, ArcLab at PolyU, HK, China
ran-sn.duan@connect.polyu.hk
rduan036@gmail.com

### Acknowledgement:
I would like to thank Mr. Shengyang Chen(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk, this code is partly refered to his work FLVIS: https://github.com/Ttoto/FLVIS
