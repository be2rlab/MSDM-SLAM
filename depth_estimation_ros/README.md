# Scaled_Metric_Depth_Estimation_ROS
A ROS node for generating scaled metric depth estimation using MiDaS from https://github.com/isl-org/MiDaS and to generate point cloud of the estimated metric depth point cloud. 

To use it PyTorch +1.7 should be there with CUDA +11.0, PyTorch should be installed in this way:
~~~
cd ~/
wget https://download.pytorch.org/libtorch/cu110/libtorch-cxx11-abi-shared-with-deps-1.7.0%2Bcu110.zip
unzip libtorch-cxx11-abi-shared-with-deps-1.7.0+cu110.zip
~~~

Now from ROS workspace:
~~~
https://github.com/maliksyria/depth_estimation_ros.git
~~~
Then :
~~~
chmod +x src/depth_estimation_ros/scripts/*.py
~~~

Followed by:
~~~
catkin_make
~~~ 
Don't forget to source devel.bash file of ROS workspace.

A pre-trained traced model should be in cached ROS directory which can be done by:

~~~
cd ~/.ros
wget https://github.com/intel-isl/MiDaS/releases/download/v2_1/model-small-traced.pt
~~~
# Run
~~~
roslaunch midas_cpp midas_cpp_xyzrgb.launch input_topic:="/input/image" gt_topic:="/ground_truth_depth" camera_info_in:="/image/camera_info"
~~~
