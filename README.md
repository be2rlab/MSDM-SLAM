# MSDM-SLAM
This repository represnets a 3D DNN-based Metric Semantic Dense Mapping pipeline and a Visual Inertial SLAM system that can be run on a ground mobile robot for the following tasks:
- An accurate automatic indoor dense 3D mapping with objects' semantic annotations. 
- A robust and accurate SLAM system with enhanced version of ORB-SLAM3.
- Supporting 2D LiDAR mapping and navigation on occupancy grid. 

One of the nodes in this pipeline is a ROS node for generating scaled metric depth estimation using MiDaS from https://github.com/isl-org/MiDaS and to generate point cloud of the estimated metric depth point cloud. Scaling process can be done using both of depth values from RGB-D camera or from features from Visual SLAM system.

To use that node PyTorch +1.7 should be there with CUDA +11.0.

### Run Depth Only
~~~
roslaunch midas_cpp midas_cpp_xyzrgb.launch input_topic:="/input/image" gt_topic:="/ground_truth_depth" camera_info_in:="/image/camera_info"
~~~
### Run Segmentation only 

~~~
roslaunch midas_cpp midas_cpp_seg.launch input_topic:="/input/image"  image_input_original_topic:="/input/image_color" segmentation_topic:="/segmented"
~~~
### Run Segmentation and Depth rescale with SLAM features
~~~
roslaunch midas_cpp midas_cpp_features_seg.launch input_topic:="/input/image"  camera_info_in:="/input/camera_info" map_topic:="/input/map_point" pose_topic:="/input/pose" image_input_original_topic:="/input/image_color" segmentation_topic:="/segmented"
~~~

### ORB_SLAM3
**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

The [Changelog](https://github.com/UZ-SLAMLab/ORB_SLAM3/Changelog.md) describes the features of each version.

ORB-SLAM3 is the first real-time SLAM library able to perform **Visual, Visual-Inertial and Multi-Map SLAM** with **monocular, stereo and RGB-D** cameras, using **pin-hole and fisheye** lens models. In all sensor configurations, ORB-SLAM3 is as robust as the best systems available in the literature, and significantly more accurate. 

We provide examples to run ORB-SLAM3 in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) using stereo or monocular, with or without IMU, and in the [TUM-VI dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) using fisheye stereo or monocular, with or without IMU. Videos of some example executions can be found at [ORB-SLAM3 channel](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q).

This software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) developed by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a>
