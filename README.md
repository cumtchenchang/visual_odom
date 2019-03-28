## Impression
1. No BA optimizatin in this project
2. Buckect and circle match is a good idea. Bucket can be seen as a type of features extraction such as image segmentated into blocks.
3. function distinguishNewPoints() will drag down the real-time nature of the system
   1) add new features, and check every feature is already exist.
4. Not use keyframe strategy

## Stereo Visual Odometry

This repository is C++ OpenCV implementation of SOFT (Stereo Odometry based on careful Feature selection and Tracking)

Original Paper: https://lamor.fer.hr/images/50020776/Cvisic2017.pdf

Demo vedio: https://www.youtube.com/watch?v=Z3S5J_BHQVw&t=17s

![alt text](https://github.com/ZhenghaoFei/visual_odom/blob/master/images/features.png "features")
![Image text](https://raw.githubusercontent.com/cumtchenchang/visual_odom/master/images/euroc_trajectory.png）

![alt text](https://github.com/ZhenghaoFei/visual_odom/blob/master/images/trajectory.png "trajectory")


### Requirements
[OpenCV 3.0](https://opencv.org/)

[Eigen 3.34](https://eigen.tuxfamily.org/dox/GettingStarted.html)

### Dataset
Tested on [KITTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) odometry dataset

### Compile & Run
```bash
git clone https://github.com/ZhenghaoFei/visual_odom.git
```
Change **EIGENPATH** in visual_odometry/CMakeLists.txt

The system use **Camera Parameters** in calibration/xx.yaml, put your own camera parameters in the same format and pass the path when you run.

```bash
mkdir build
cd build
cmake ..
make -j4
./run /PathtoKITTI/sequences/00/ ../calibration/kitti00.yaml
```
### Reference code
1. [Monocular visual odometry algorithm](https://github.com/avisingh599/mono-vo/blob/master/README.md)

2. [Matlab implementation of SOFT](https://github.com/Mayankm96/Stereo-Odometry-SOFT/blob/master/README.md)
