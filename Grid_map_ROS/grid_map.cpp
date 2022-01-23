#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include<opencv2/core/core.hpp>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

// parameters
float scale_factor = 3;
float resize_factor = 5;
float cloud_max_x = 10;
float cloud_min_x = -10.0;
float cloud_max_z = 16;
float cloud_min_z = -5;
float free_thresh = 0.55;
float occupied_thresh = 0.50;
float thresh_diff = 0.01;
int visit_thresh = 0;
float upper_left_x = -1.5;
float upper_left_y = -2.5;
const int resolution = 10;
unsigned int use_local_counters = 0;

float grid_max_x, grid_min_x,grid_max_z, grid_min_z;
cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
float norm_factor_x, norm_factor_z;
int h, w;
unsigned int n_kf_received;
bool loop_closure_being_processed = false;
ros::Publisher pub_grid_map;
ros::Publisher pub_traj_map;
nav_msgs::OccupancyGrid grid_map_msg;
bool first_msg = true;

float kf_pos_x, kf_pos_z;
int kf_pos_grid_x, kf_pos_grid_z;

using namespace std;
void ptsKFCallback(const sensor_msgs::PointCloud::ConstPtr& MapPoints, const nav_msgs::Odometry::ConstPtr& Kf_pose, const sensor_msgs::Image::ConstPtr& Img);

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud);
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose);
void saveMap(unsigned int id = 0);
void ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts);
void parseParams(int argc, char **argv);
void printParams();
void showGridMap(unsigned int id = 0);
void getMixMax(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose,
	geometry_msgs::Point& min_pt, geometry_msgs::Point& max_pt);
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
	cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);
void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
void getGridMap();

int main(int argc, char **argv){
	ros::init(argc, argv, "Monosub");
	ros::start();

	parseParams(argc, argv);
	printParams();

	grid_max_x = cloud_max_x*scale_factor;
	grid_min_x = cloud_min_x*scale_factor;
	grid_max_z = cloud_max_z*scale_factor;
	grid_min_z = cloud_min_z*scale_factor;
	printf("grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);

	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z;
	w = grid_res_x;
	printf("grid_size: (%d, %d)\n", h, w);
	n_kf_received = 0;

	global_occupied_counter.create(h, w, CV_32SC1);
	global_visit_counter.create(h, w, CV_32SC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

	grid_map_msg.data.resize(h*w);
	grid_map_msg.info.width = w;
	grid_map_msg.info.height = h;
	grid_map_msg.info.resolution = 1.0/scale_factor;

	grid_map_int = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data()));

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	grid_map_thresh_resized.create(h*resize_factor, w*resize_factor, CV_8UC1);
	printf("output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);

	local_occupied_counter.create(h, w, CV_32SC1);
	local_visit_counter.create(h, w, CV_32SC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
	printf("norm_factor_x: %f\n", norm_factor_x);
	printf("norm_factor_z: %f\n", norm_factor_z);

	ros::NodeHandle nodeHandler;

	// Original Grid Map

	//ros::Subscriber sub_pts_and_pose = nodeHandler.subscribe("pts_and_pose", 1000, ptCallback);
	//ros::Subscriber sub_all_kf_and_pts = nodeHandler.subscribe("all_kf_and_pts", 1000, loopClosingCallback);
	message_filters::Subscriber<sensor_msgs::Image> Image_sub(nodeHandler, "/vslam_stereo_inertial/KF_DebugImage", 1000);


	message_filters::Subscriber<sensor_msgs::PointCloud> MapPoints_sub(nodeHandler, "/vslam_stereo_inertial/KF_MapPoints", 1000);
    message_filters::Subscriber<nav_msgs::Odometry> KF_pose_sub(nodeHandler, "/vslam_stereo_inertial/KF_Pose", 1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, nav_msgs::Odometry, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1000),  MapPoints_sub, KF_pose_sub, Image_sub);
    sync.registerCallback(boost::bind(&ptsKFCallback,_1,_2, _3));
	

	pub_grid_map = nodeHandler.advertise<nav_msgs::OccupancyGrid>("grid_map", 1000);
	pub_traj_map = nodeHandler.advertise<sensor_msgs::Image>("trajectory_grid_map", 1000);
	//ros::Subscriber sub_cloud = nodeHandler.subscribe("cloud_in", 1000, cloudCallback);
	//ros::Subscriber sub_kf = nodeHandler.subscribe("camera_pose", 1000, kfCallback);
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	ros::spin();
	ros::shutdown();
	cv::destroyAllWindows();
	saveMap();
	
	return 0;
}

void ptsKFCallback(const sensor_msgs::PointCloud::ConstPtr& MapPoints, const nav_msgs::Odometry::ConstPtr& Kf_pose, const sensor_msgs::Image::ConstPtr& Img){
	
	std_msgs::Header h1 = MapPoints->header;
	std_msgs::Header h2 = Kf_pose->header;
	/*
	{
		cv_bridge::CvImageConstPtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvShare(Img);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat imgs;
		auto sz = cv_ptr->image.size();
		sz.height *=0.3;
		sz.width *=0.3;
		cv::resize(cv_ptr->image,imgs,sz);
		cv::imshow("Image", imgs);
		cv::waitKey(1);
	}*/
	//if (loop_closure_being_processed){ return; }

	geometry_msgs::PoseArray pts_and_pose;
	pts_and_pose.header.seq = h2.seq;
	geometry_msgs::Pose temp;
	temp.position = Kf_pose->pose.pose.position;
	temp.orientation = Kf_pose->pose.pose.orientation;
	
	Eigen::Quaterniond Q(temp.orientation.w,
						 temp.orientation.x,
						 temp.orientation.y,
						 temp.orientation.z);
	
	

	Eigen::Matrix3d Rcw = Q.toRotationMatrix();
	Eigen::Vector3d Pcw(temp.position.x,
	temp.position.y,
	temp.position.z);

	Eigen::Vector3d Pwc = - Rcw.transpose() * Pcw;

	temp.position.x = Pwc.x();
	temp.position.z = Pwc.y();
	temp.position.y = Pwc.z();
	
	Eigen::Quaterniond Qwc(Rcw.transpose());
	temp.orientation.w = Qwc.w();
	temp.orientation.x = Qwc.x();
	temp.orientation.y = Qwc.y();
	temp.orientation.z = Qwc.z();
	//cout << temp.position;
	if (first_msg){
		
		grid_map_msg.info.origin.position.x = temp.position.y + 0.97 *cloud_min_z ;
		grid_map_msg.info.origin.position.y = temp.position.x - 1.05 *cloud_min_x;
		grid_map_msg.info.origin.position.z = temp.position.z;
		
		grid_map_msg.info.origin.orientation.w = temp.orientation.w;
		grid_map_msg.info.origin.orientation.x = temp.orientation.y;
		grid_map_msg.info.origin.orientation.y = temp.orientation.z;
		grid_map_msg.info.origin.orientation.z = temp.orientation.x;
		first_msg =false;
		cout << temp << endl;
		cout << grid_map_msg.info.origin << endl;
	}

	pts_and_pose.poses.push_back(temp);
	
	for(auto mp:MapPoints->points){
		Eigen::Vector3d point(mp.x,mp.y,mp.z);
		if(point.norm()==0)
			continue;
		if(mp.z <-2 || mp.z > 1)
			continue;
		geometry_msgs::Pose tempPt;
		tempPt.position.x = mp.x;
		tempPt.position.z = mp.y;
		tempPt.position.y = mp.z;

		
		pts_and_pose.poses.push_back(tempPt);
	}
	//auto foo = std::const_pointer_cast<geometry_msgs::PoseArray>(goo);

	boost::shared_ptr<geometry_msgs::PoseArray> pts_and_pose_temp = boost::make_shared<geometry_msgs::PoseArray>(pts_and_pose);	
	updateGridMap(pts_and_pose_temp);
	//auto trajMsg = grid_map_msg; // Copy of masssege
	/*static std::vector<cv::Point> trj;
	cv::Mat addTraj = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data()));
	{
		auto pos_x = temp.position.x*scale_factor;
		auto pos_z = temp.position.z*scale_factor;

		auto pos_grid_x = int(floor((pos_x - grid_min_x) * norm_factor_x));
		auto pos_grid_z = int(floor((pos_z - grid_min_z) * norm_factor_z));
		trj.push_back(cv::Point(pos_grid_x,  pos_grid_z));
		for(size_t i = 0; i < trj.size();i++){
			cv::circle(addTraj, trj[i], 2, cv::Scalar(100), -1);
			if(i!=0){
				cv::line(addTraj, trj[i-1], trj[i], cv::Scalar(100), 2);
			}
		}
	}*/
	grid_map_msg.info.map_load_time = ros::Time::now();
	pub_grid_map.publish(grid_map_msg);
	
	//showGridMap(); show grid_map_msg and grid_map_thresh_resize
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud){
	ROS_INFO("I heard: [%s]{%d}", pt_cloud->header.frame_id.c_str(),
		pt_cloud->header.seq);
}
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose){
	ROS_INFO("I heard: [%s]{%d}", camera_pose->header.frame_id.c_str(),
		camera_pose->header.seq);
}
void saveMap(unsigned int id) {
	printf("saving maps with id: %u\n", id);
	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (id > 0) {
		cv::imwrite("results//grid_map_" + to_string(id) + ".jpg", grid_map);
		cv::imwrite("results//grid_map_thresh_" + to_string(id) + ".jpg", grid_map_thresh);
		cv::imwrite("results//grid_map_thresh_resized" + to_string(id) + ".jpg", grid_map_thresh_resized);
		
		std::vector<int> compression_params; // Stores the compression parameters
		compression_params.push_back(CV_IMWRITE_PXM_BINARY); // Set to PXM compression
		compression_params.push_back(0); // Set type of PXM in our case PGM
		const std::string imageFilename = "results//GridMap.pgm"; // Some file name - C++ requires an std::string

		cv::imwrite(imageFilename, grid_map_thresh, compression_params); // Write matrix to file

	}
	else {
		cv::imwrite("results//grid_map.jpg", grid_map);
		cv::imwrite("results//grid_map_thresh.jpg", grid_map_thresh);
		cv::imwrite("results//grid_map_thresh_resized.jpg", grid_map_thresh_resized);
		std::vector<int> compression_params; // Stores the compression parameters
		compression_params.push_back(CV_IMWRITE_PXM_BINARY); // Set to PXM compression
		compression_params.push_back(0); // Set type of PXM in our case PGM
		const std::string imageFilename = "results//GridMap.pgm"; // Some file name - C++ requires an std::string

		cv::imwrite(imageFilename, grid_map_thresh, compression_params); // Write matrix to file
	}

}
void ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	//if (pts_and_pose->header.seq==0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	printf("Received exit message\n");
	//	ros::shutdown();
	//	exit(0);
	//}
//	if (!got_start_time) {
//#ifdef COMPILEDWITHC11
//		start_time = std::chrono::steady_clock::now();
//#else
//		start_time = std::chrono::monotonic_clock::now();
//#endif
//		got_start_time = true;
//	}

	if (loop_closure_being_processed){ return; }

	updateGridMap(pts_and_pose);

	grid_map_msg.info.map_load_time = ros::Time::now();

	pub_grid_map.publish(grid_map_msg);
}
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts){
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	//if (all_kf_and_pts->header.seq == 0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	ros::shutdown();
	//	exit(0);
	//}
	loop_closure_being_processed = true;
	resetGridMap(all_kf_and_pts);
	loop_closure_being_processed = false;
}

void getMixMax(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose,
	geometry_msgs::Point& min_pt, geometry_msgs::Point& max_pt) {

	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
	max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
	for (unsigned int i = 0; i < pts_and_pose->poses.size(); ++i){
		const geometry_msgs::Point& curr_pt = pts_and_pose->poses[i].position;
		if (curr_pt.x < min_pt.x) { min_pt.x = curr_pt.x; }
		if (curr_pt.y < min_pt.y) { min_pt.y = curr_pt.y; }
		if (curr_pt.z < min_pt.z) { min_pt.z = curr_pt.z; }

		if (curr_pt.x > max_pt.x) { max_pt.x = curr_pt.x; }
		if (curr_pt.y > max_pt.y) { max_pt.y = curr_pt.y; }
		if (curr_pt.z > max_pt.z) { max_pt.z = curr_pt.z; }
	}
}
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied, 
	cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z) {
	float pt_pos_x = curr_pt.x*scale_factor;
	float pt_pos_z = curr_pt.z*scale_factor;

	int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));


	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;

	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(pt_pos_grid_z, pt_pos_grid_x);
	pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;

	//cout << "----------------------" << endl;
	//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

	// Get all grid cell that the line between keyframe and map point pass through
	int x0 = kf_pos_grid_x;
	int y0 = kf_pos_grid_z;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_z;
	bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep){
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1){
		swap(x0, x1);
		swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x){
		if (steep) {
			++visited.at<int>(x, y);
		}
		else {
			++visited.at<int>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5){
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z) {
	unsigned int end_id = start_id + n_pts;
	if (use_local_counters) {
		local_map_pt_mask.setTo(0);
		local_occupied_counter.setTo(0);
		local_visit_counter.setTo(0);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
		for (int row = 0; row < h; ++row){
			for (int col = 0; col < w; ++col){
				if (local_map_pt_mask.at<uchar>(row, col) == 0) {
					local_occupied_counter.at<int>(row, col) = 0;
				}
				else {
					local_occupied_counter.at<int>(row, col) = local_visit_counter.at<int>(row, col);
				}
			}
		}
		global_occupied_counter += local_occupied_counter;
		global_visit_counter += local_visit_counter;
	}
	else {
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
	}
}

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose){

	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	//printf("max_pt: %f, %f\t min_pt: %f, %f\n", max_pt.x*scale_factor, max_pt.z*scale_factor, 
	//	min_pt.x*scale_factor, min_pt.z*scale_factor);

	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;

	//printf("Received frame %u \n", pts_and_pose->header.seq);
	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;

	kf_pos_x = kf_location.x*scale_factor;
	kf_pos_z = kf_location.z*scale_factor;

	kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
	kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
		return;

	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
		return;
	++n_kf_received;
	unsigned int n_pts = pts_and_pose->poses.size() - 1;
	//printf("Processing key frame %u and %u points\n",n_kf_received, n_pts);
	processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);
	getGridMap();
	{
		cv::Mat traj = grid_map_thresh_resized.clone();
		static std::vector<cv::Point> trj;
		trj.push_back(cv::Point(kf_pos_grid_x,kf_pos_grid_z));
		for(size_t i=0;i<trj.size();i++){
			cv::circle(traj,trj[i],2,cv::Scalar(0),-1);
			if(i!=0)
				cv::line(traj, trj[i-1], trj[i],cv::Scalar(0), 2);
		}
		cv::imshow("Trajectory", traj);
		cv::waitKey(1);
	}
	
	//showGridMap(pts_and_pose->header.seq);
	//cout << endl << "Grid map saved!" << endl;
}

void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts){
	global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int) (all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int) (all_kf_and_pts->poses[0].position.z) != n_kf) {
		printf("resetGridMap :: Unexpected formatting in the keyframe count element\n");
		return;
	}
	printf("Resetting grid map with %d key frames\n", n_kf);
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id){
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts) {
			printf("resetGridMap :: Unexpected formatting in the point count element for keyframe %d\n", kf_id);
			return;
		}
		float kf_pos_x = kf_location.x*scale_factor;
		float kf_pos_z = kf_location.z*scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			continue;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			continue;

		if (id + n_pts >= all_kf_and_pts->poses.size()) {
			printf("resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
				kf_id, n_pts, all_kf_and_pts->poses.size(), id + n_pts);
			return;
		}
		processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
		id += n_pts;
	}	
	getGridMap();
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
	double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	printf("Done. Time taken: %f secs\n", ttrack);
	pub_grid_map.publish(grid_map_msg);
	showGridMap(all_kf_and_pts->header.seq);
}

void getGridMap() {
	for (int row = 0; row < h; ++row){
		for (int col = 0; col < w; ++col){
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

			if (visits <= visit_thresh){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
			}
			if (grid_map.at<float>(row, col) >= free_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else {
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
		}
	}
	cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}
void showGridMap(unsigned int id) {
	cv::imshow("grid_map_msg", cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data())));
	cv::imshow("grid_map_thresh_resized", grid_map_thresh_resized);
	//cv::imshow("grid_map", grid_map);
	int key = cv::waitKey(1) % 256;
	if (key == 27) {
		cv::destroyAllWindows();
		ros::shutdown();
		exit(0);
	}
	else if (key == 'f') {
		free_thresh -= thresh_diff;
		if (free_thresh <= occupied_thresh){ free_thresh = occupied_thresh + thresh_diff; }

		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'F') {
		free_thresh += thresh_diff;
		if (free_thresh > 1){ free_thresh = 1; }
		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'o') {
		occupied_thresh -= thresh_diff;
		if (free_thresh < 0){ free_thresh = 0; }
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 'O') {
		occupied_thresh += thresh_diff;
		if (occupied_thresh >= free_thresh){ occupied_thresh = free_thresh - thresh_diff; }
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 's') {
		saveMap(id);
	}
}

void parseParams(int argc, char **argv) {
	int arg_id = 1;
	if (argc > arg_id){
		scale_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		resize_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		cloud_max_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		cloud_min_x = atof(argv[arg_id++]);
	}	
	if (argc > arg_id){
		cloud_max_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		cloud_min_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		free_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		occupied_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id){
		use_local_counters = atoi(argv[arg_id++]);
	}
	if (argc > arg_id){
		visit_thresh = atoi(argv[arg_id++]);
	}
}

void printParams() {
	printf("Using params:\n");
	printf("scale_factor: %f\n", scale_factor);
	printf("resize_factor: %f\n", resize_factor);
	printf("cloud_max: %f, %f\t cloud_min: %f, %f\n", cloud_max_x, cloud_max_z, cloud_min_x, cloud_min_z);
	printf("free_thresh: %f\n", free_thresh);
	printf("occupied_thresh: %f\n", occupied_thresh);
	printf("use_local_counters: %d\n", use_local_counters);
	printf("visit_thresh: %d\n", visit_thresh);

}

