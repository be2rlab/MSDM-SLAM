//
// Created by vuong on 23/01/2022.
//
/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

// FROM ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// FROM System
#include <chrono>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <queue>
#include <thread>
#include <vector>

// FROM ORB_SLAM3
#include <ImuTypes.h>
#include <System.h>
#include <node.h>

using namespace std;

class ImuGrabber {
 public:
  ImuGrabber() = default;

  void GrabImu(const sensor_msgs::ImuConstPtr& imu_msg);

  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutex;
};

class VIO : public node {
 public:
  VIO(ORB_SLAM3::System::eSensor sensor,
      ros::NodeHandle& node_handle,
      image_transport::ImageTransport& image_transport,
      std::string strOutput = std::string());

  ~VIO();

  void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);

  void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);

  static cv::Mat GetImage(const sensor_msgs::ImageConstPtr& img_msg);

  void SyncWithImu();

  ImuGrabber* mpImuGb;

 public:
  void setmbClahe(bool mbClahe);

  void SavingTrajectory();

 private:
  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;

 private:
  // Variables for preprocessing images before passing to ORB_SLAM3
  bool mbResize = false;
  bool mbClahe = false;
  bool mbRectify = true;

 private:
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ORB_SLAM3");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport image_transport(nh);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  // Some static parameters
  std::string strVocFile;
  std::string strSettingsFile;
  std::string strOutput;
  bool bEqual = false;

  // Retrieve parameters
  nh.getParam("Params", strSettingsFile);
  nh.getParam("Do_Equalize", bEqual);
  nh.getParam("Output_name", strOutput);

  auto* imugb = new ImuGrabber();

  VIO mVIO(
      ORB_SLAM3::System::eSensor::IMU_STEREO, nh, image_transport, strOutput);

  mVIO.mpImuGb = imugb;

  mVIO.setmbClahe(bEqual);

  mVIO.Init();

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu =
      nh.subscribe("/imu", 1000, &ImuGrabber::GrabImu, imugb);
  ros::Subscriber sub_img_left =
      nh.subscribe("/camera/left/image_raw", 100, &VIO::GrabImageLeft, &mVIO);
  ros::Subscriber sub_img_right =
      nh.subscribe("/camera/right/image_raw", 100, &VIO::GrabImageRight, &mVIO);

  std::thread sync_thread(&VIO::SyncWithImu, &mVIO);

  ros::spin();

  mVIO.SavingTrajectory();

  return 0;
}

void VIO::GrabImageLeft(const sensor_msgs::ImageConstPtr& img_msg) {
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty()) imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void VIO::GrabImageRight(const sensor_msgs::ImageConstPtr& img_msg) {
  mBufMutexRight.lock();
  if (!imgRightBuf.empty()) imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

cv::Mat VIO::GetImage(const sensor_msgs::ImageConstPtr& img_msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void VIO::SyncWithImu() {
  const double maxTimeDiff = 0.01;
  while (ros::ok()) {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty() && !imgRightBuf.empty() &&
        !mpImuGb->imuBuf.empty()) {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      this->mBufMutexRight.lock();
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff ||
          (tImRight - tImLeft) > maxTimeDiff)
        continue;

      if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec()) continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if (!mpImuGb->imuBuf.empty()) {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() &&
               mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(
              (float)mpImuGb->imuBuf.front()->linear_acceleration.x,
              (float)mpImuGb->imuBuf.front()->linear_acceleration.y,
              (float)mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr((float)mpImuGb->imuBuf.front()->angular_velocity.x,
                          (float)mpImuGb->imuBuf.front()->angular_velocity.y,
                          (float)mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.emplace_back(acc, gyr, t);
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if (mbClahe) {
        mClahe->apply(imLeft, imLeft);
        mClahe->apply(imRight, imRight);
      }
      mORB_SLAM3->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
      Update();
      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

VIO::VIO(ORB_SLAM3::System::eSensor sensor,
         ros::NodeHandle& node_handle,
         image_transport::ImageTransport& image_transport,
         std::string strOutput)
    : node(sensor, node_handle, image_transport, strOutput) {}

void VIO::setmbClahe(bool bClahe) { VIO::mbClahe = bClahe; }

VIO::~VIO() {
  // Release memory
  delete mORB_SLAM3;
  delete mpLocalMapping;
  delete mpMapDrawer;
  delete mpAtlas;
}

void VIO::SavingTrajectory() {
  // Stop all threads
  mORB_SLAM3->Shutdown();
  ROS_INFO("Saving trajectory...");
  // Save trajectory
  if (mSensor == ORB_SLAM3::System::MONOCULAR ||
      mSensor == ORB_SLAM3::System::IMU_MONOCULAR) {
    mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  } else {
    if (strOutputFile.empty()) {
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
      mORB_SLAM3->SaveTrajectoryTUM("FrameTrajectory.txt");
    } else {
      mORB_SLAM3->SaveKeyFrameTrajectoryTUM("kf_" + strOutputFile + ".txt");
      mORB_SLAM3->SaveTrajectoryTUM("f_" + strOutputFile + ".txt");
    }
  }
  ROS_INFO("Saved trajectory!");
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& imu_msg) {
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
}
