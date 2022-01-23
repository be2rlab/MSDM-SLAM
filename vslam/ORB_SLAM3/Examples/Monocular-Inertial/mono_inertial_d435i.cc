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
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>

#include "ImuTypes.h"

using namespace std;

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageFilenames,
                vector<double>& vTimestamps);

void LoadIMU(const string& strImuPath,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro);

double ttrack_tot = 0;
int main(int argc, char* argv[]) {
  if (argc < 5) {
    cerr << endl
         << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings "
            "path_to_sequence_folder_1 (path_to_image_folder_2... "
            "path_to_image_folder_N "
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
  vector<vector<string> > vstrImageFilenames;
  vector<vector<double> > vTimestampsCam;
  vector<vector<cv::Point3f> > vAcc, vGyro;
  vector<vector<double> > vTimestampsImu;
  vector<int> nImages;
  vector<int> nImu;
  vector<int> first_imu(num_seq, 0);

  vstrImageFilenames.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  vAcc.resize(num_seq);
  vGyro.resize(num_seq);
  vTimestampsImu.resize(num_seq);
  nImages.resize(num_seq);
  nImu.resize(num_seq);

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "...";

    string pathSeq(argv[(seq) + 3]);

    LoadImages(pathSeq, vstrImageFilenames[seq], vTimestampsCam[seq]);
    cout << "LOADED!" << endl;

    cout << "Loading IMU for sequence " << seq << "...";
    LoadIMU(pathSeq, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
    cout << "LOADED!" << endl;

    nImages[seq] = vstrImageFilenames[seq].size();
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

  cout.precision(17);

  /*cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl;
  cout << "IMU data in the sequence: " << nImu << endl << endl;*/

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(
      argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  int proccIm = 0;
  for (seq = 0; seq < num_seq; seq++) {
    // Main loop
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read image from file
      im = cv::imread(vstrImageFilenames[seq][ni], CV_LOAD_IMAGE_UNCHANGED);

      double tframe = vTimestampsCam[seq][ni];

      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageFilenames[seq][ni]
             << endl;
        return 1;
      }
      cv::resize(im, im, cv::Size(640, 360));
      // Load imu measurements from previous frame
      vImuMeas.clear();

      if (ni > 0) {
        // cout << "t_cam " << tframe << endl;

        while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]) {
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
      }

      // Apply calibration on IMU for d435i Camera
      // Eigen::Vector4d imu_gyro,imu_acc;
      // Eigen::Matrix<double,4,4> gyro_calib,acc_calib;
      // acc_calib <<      1.01299989e+00,  1.62997451e-02, -1.65678188e-02,
      // -2.38034371e-02,
      //                   9.05588327e-04,  1.01791656e+00, -8.24022759e-03,
      //                   -9.57686007e-02,
      //                   -2.26753242e-02,  6.72622863e-03,  1.01643240e+00, 2.40073770e-01,
      //                    0.000000f,       0.000000f,
      //                    0.000000f,       1.000000f;

      // gyro_calib <<      1.00000000f,  0.00000000f,  0.00000000f,
      // -6.76363852e-05,
      //                    0.00000000f,  1.00000000f,  0.00000000f,
      //                    -9.54246752e-06, 0.00000000f,
      //                    0.00000000f,  1.00000000f, -1.75042805e-05,
      //                    0.000000f,    0.000000f,    0.000000f,    1.000000f;

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
      /*cout << "first imu: " << first_imu << endl;
      cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl;
      cout << "size vImu: " << vImuMeas.size() << endl;*/
#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      // cout << "tframe = " << tframe << endl;
      SLAM.TrackMonocular(
          im, tframe, vImuMeas);  // TODO change to monocular_inertial

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
      ttrack_tot += ttrack;
      // std::cout << "ttrack: " << ttrack << std::endl;

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
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
  } else {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  }

  return 0;
}

void LoadImages(const string& strPathToSequence,
                vector<string>& vstrImageFilenames,
                vector<double>& vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/d455_times_color.txt";
  fTimes.open(strPathTimeFile.c_str());
  string strPrefixLeft = strPathToSequence + "/color_d455/";
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (s[0] == '#') continue;
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t / 1e9);
      vstrImageFilenames.push_back(strPrefixLeft + ss.str() + ".png");
    }
  }
}

void LoadIMU(const string& strPathToSequence,
             vector<double>& vTimeStamps,
             vector<cv::Point3f>& vAcc,
             vector<cv::Point3f>& vGyro) {
  ifstream fImu;
  string strImuPath = strPathToSequence + "/d455_imu.txt";
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

      vTimeStamps.push_back(data[0] / 1e9);
      vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
      vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
    }
  }
}
