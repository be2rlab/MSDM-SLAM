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
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string& strAssociationFilename,
                vector<string>& vstrImageFilenamesRGB,
                vector<string>& vstrImageFilenamesD,
                vector<double>& vTimestamps);

int main(int argc, char** argv) {
  // bool bFilename = false;
  if (argc < 4) {
    cerr << endl
         << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings "
            "path_to_sequence_1 path_to_sequence_2... (file_name)"
         << endl;
    return 1;
  }
  const int num_seq = (argc - 3 - 1);
  bool bFileName = true;
  // if (argc == 5)
  //     bFilename = true;
  // Retrieve paths to images
  vector<vector<string> > vstrImageFilenamesRGB;
  vector<vector<string> > vstrImageFilenamesD;
  vector<vector<double> > vTimestamps;
  vector<int> nImages;
  vstrImageFilenamesRGB.resize(num_seq);
  vstrImageFilenamesD.resize(num_seq);
  vTimestamps.resize(num_seq);
  nImages.resize(num_seq);

  int tot_images = 0;
  for (int seq = 0; seq < num_seq; seq++) {
    string strAssociationFilename = string(argv[3 + seq]);
    cout << "Loading seq " << seq << endl;
    LoadImages(strAssociationFilename,
               vstrImageFilenamesRGB[seq],
               vstrImageFilenamesD[seq],
               vTimestamps[seq]);

    // Check consistency in the number of images and depthmaps
    // int nImages = vstrImageFilenamesRGB.size();
    if (vstrImageFilenamesRGB.empty()) {
      cerr << endl << "No images found in provided path." << endl;
      return 1;
    } else if (vstrImageFilenamesD.size() != vstrImageFilenamesRGB.size()) {
      cerr << endl << "Different number of images for rgb and depth." << endl;
      return 1;
    }
    cout << "Loaded! Seq " << seq << ": " << vstrImageFilenamesRGB[seq].size()
         << " imgs" << endl;
    nImages[seq] = vstrImageFilenamesRGB[seq].size();
    tot_images += nImages[seq];
  }
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;

  // Main loop
  cv::Mat imRGB, imD;
  for (int seq = 0; seq < num_seq; seq++) {
    int proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read image and depthmap from file
      imRGB = cv::imread(
          string(argv[3 + seq]) + "/" + vstrImageFilenamesRGB[seq][ni],
          cv::IMREAD_UNCHANGED);
      imD =
          cv::imread(string(argv[3 + seq]) + "/" + vstrImageFilenamesD[seq][ni],
                     cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[seq][ni];

      // cout << "Start processing sequence ..." << endl;
      // cout << "Images in the sequence: " << nImages[seq] << endl << endl;

      if (imRGB.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(argv[3] + seq) << "/"
             << vstrImageFilenamesRGB[seq][ni] << endl;
        return 1;
      }

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the image to the SLAM system
      SLAM.TrackRGBD(imRGB, imD, tframe);

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
        T = vTimestamps[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestamps[seq][ni - 1];

      if (ttrack < T) usleep((T - ttrack) * 1e6);
    }
    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }
  // Stop all threads
  SLAM.Shutdown();

  // // Tracking time statistics
  // sort(vTimesTrack.begin(),vTimesTrack.end());
  // float totaltime = 0;
  // for(int ni=0; ni<nImages; ni++)
  // {
  //     totaltime+=vTimesTrack[ni];
  // }
  // cout << "-------" << endl << endl;
  // cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  // cout << "mean tracking time: " << totaltime/nImages << endl;

  if (bFileName) {
    string filename = argv[argc - 1];
    SLAM.SaveTrajectoryTUM("f_ " + filename + ".txt");
    SLAM.SaveKeyFrameTrajectoryTUM("kf_ " + filename + ".txt");
  } else {
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }
  // Save camera trajectory

  return 0;
}

void LoadImages(const string& strAssociationFilename,
                vector<string>& vstrImageFilenamesRGB,
                vector<string>& vstrImageFilenamesD,
                vector<double>& vTimestamps) {
  ifstream fAssociation;
  string strColor = strAssociationFilename + "/color.txt";
  fAssociation.open(strColor.c_str());
  while (!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
    }
  }
  fAssociation.close();
  string strDepth = strAssociationFilename + "/aligned_depth.txt";
  fAssociation.open(strDepth.c_str());
  while (!fAssociation.eof()) {
    string s;
    getline(fAssociation, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sD;
      ss >> t;
      ss >> sD;
      vstrImageFilenamesD.push_back(sD);
    }
  }
  fAssociation.close();
}
