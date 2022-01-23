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

#include <System.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>

#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string& strPathFolder,
                vector<string>& vstrImageLeft,
                vector<double>& vTimeStamps);

void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro);

int main(int argc, char** argv) {
  if (argc < 5) {
    cerr << endl
         << "Usage: ./mono_inertial_openloris path_to_vocabulary "
            "path_to_settings path_to_sequence_folder_1 "
            "(path_to_image_folder_2 ... path_to_image_folder_N) "
         << endl;
    return 1;
  }

  const int num_seq = (argc - 4);
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = true;
  string file_name;
  if (bFileName) {
    file_name = string(argv[argc - 1]);
    cout << "file name: " << file_name << endl;
  }

  // Load all sequences:
  int seq;
  vector<vector<string> > vstrImageLeft;
  vector<vector<double> > vTimestampsCam;
  vector<vector<cv::Point3f> > vAcc, vGyro;
  vector<vector<double> > vTimestampsImu;
  vector<int> nImages;
  vector<int> nImu;
  vector<int> first_imu(num_seq, 0);

  vstrImageLeft.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);
  vTimestampsImu.resize(num_seq);
  nImages.resize(num_seq);
  nImu.resize(num_seq);
  // first_imu.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "...";

    string pathSeq(argv[(seq) + 3]);

    string pathImu = pathSeq + "/t265_imu.txt";

    LoadImages(pathSeq, vstrImageLeft[seq], vTimestampsCam[seq]);
    cout << endl;
    cout << "Total images: " << vstrImageLeft[seq].size() << endl;
    cout << "LOADED!" << endl;

    cout << "Loading IMU for sequence " << seq << "...";
    LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
    cout << endl;
    cout << "Total IMU meas: " << vTimestampsImu[seq].size() << endl;
    cout << "LOADED!" << endl;

    nImages[seq] = vstrImageLeft[seq].size();
    tot_images += nImages[seq];
    nImu[seq] = vTimestampsImu[seq].size();

    if ((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
      cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
      return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first

    while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
      first_imu[seq]++;
    first_imu[seq]--;  // first imu measurement to be considered
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  static Eigen::Matrix4d gyro_calib, acc_calib;

  acc_calib << 1.0129998922348022, 1.6299745067954063e-02,
      -1.6567818820476532e-02, -2.3803437128663063e-02, 9.0558832744136453e-04,
      1.0179165601730347, -8.2402275875210762e-03, -9.5768600702285767e-02,
      -2.2675324231386185e-02, 6.7262286320328712e-03, 1.0164324045181274e+00,
      2.4007377028465271e-01, 0.000000, 0.000000, 0.000000, 1.000000;

  gyro_calib << 1.000000, 0.000000, 0.000000, -6.7636385210789740e-05, 0.000000,
      1.000000, 0.000000, -9.5424675237154588e-06, 0.000000, 0.000000, 1.000000,
      -1.7504280549474061e-05, 0.000000, 0.000000, 0.000000,
      1.000000000000000000000;

  cout << "Gyro instric matrix:" << endl;
  cout << gyro_calib << endl;
  cout << "Accel instric matrix:" << endl;
  cout << acc_calib << endl;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(
      argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  cv::Mat imLeft, imRight;
  int proccIm = 0;
  for (seq = 0; seq < num_seq; seq++) {
    // Seq loop
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    proccIm = 0;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_GRAYSCALE);
      // clahe
      // clahe->apply(imLeft,imLeft);
      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageLeft[seq][ni])
             << endl;
        return 1;
      }

      double tframe = vTimestampsCam[seq][ni];

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0)
        while (
            vTimestampsImu[seq][first_imu[seq]] <=
            vTimestampsCam
                [seq]
                [ni])  // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
        {
          vImuMeas.push_back(
              ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,
                                    vAcc[seq][first_imu[seq]].y,
                                    vAcc[seq][first_imu[seq]].z,
                                    vGyro[seq][first_imu[seq]].x,
                                    vGyro[seq][first_imu[seq]].y,
                                    vGyro[seq][first_imu[seq]].z,
                                    vTimestampsImu[seq][first_imu[seq]]));
          first_imu[seq]++;
        }

        // // Apply calibration on IMU for T265 Camera
        // Eigen::Vector4d imu_gyro,imu_acc;

        // for(auto& imudata:vImuMeas){
        //     imu_gyro(0) = imudata.w.x;
        //     imu_gyro(1) = imudata.w.y;
        //     imu_gyro(2) = imudata.w.z;
        //     imu_gyro(3) = 1.0f;

        //     imu_acc(0) = imudata.a.x;
        //     imu_acc(1) = imudata.a.y;
        //     imu_acc(2) = imudata.a.z;
        //     imu_acc(3) = 1.0f;

        //     imu_gyro = gyro_calib * imu_gyro;
        //     imu_acc = acc_calib * imu_acc;

        //     imudata.a.x = imu_acc(0);
        //     imudata.a.y = imu_acc(1);
        //     imudata.a.z = imu_acc(2);

        //     imudata.w.x = imu_gyro(0);
        //     imudata.w.y = imu_gyro(1);
        //     imudata.w.z = imu_gyro(2);
        // }

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the images to the SLAM system
      SLAM.TrackMonocular(imLeft, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1)
              .count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[seq] - 1)
        T = vTimestampsCam[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[seq][ni - 1];

      if (ttrack < T) usleep((T - ttrack) * 1e6);  // 1e6
    }

    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (bFileName) {
    const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
    const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
    SLAM.SaveTrajectoryTUM(f_file);
    SLAM.SaveKeyFrameTrajectoryTUM(kf_file);
  } else {
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages[0]; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages[0] / 2] << endl;
  cout << "mean tracking time: " << totaltime / proccIm << endl;

  return 0;
}

void LoadImages(const string& strPathSeq,
                vector<string>& vstrImageLeft,
                vector<double>& vTimeStamps) {
  ifstream fTimes;

  string strPathLeft = strPathSeq + "/fisheye1.txt";
  fTimes.open(strPathLeft.c_str());
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB;
      ss >> t;
      vTimeStamps.push_back(t);
      ss >> sRGB;
      vstrImageLeft.push_back(strPathSeq + "/" + sRGB);
    }
  }
  fTimes.close();
}

void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro) {
  ifstream fImu;
  fImu.open(strImuPath.c_str());
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);
    if (s[0] == '#') continue;

    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];
      int count = 0;
      while ((pos = s.find(',')) != string::npos) {
        item = s.substr(0, pos);
        data[count++] = stod(item);
        s.erase(0, pos + 1);
      }
      item = s.substr(0, pos);
      data[6] = stod(item);

      vTimeStamps.push_back(data[0]);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}
