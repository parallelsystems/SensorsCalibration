/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */
#include "Eigen/Core"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "dataConvert.hpp"
#include "logging.hpp"

#include "aruco_marker/corner_detect.hpp"
#include "calibration/pnp_solver.hpp"
#include "round_hole_board/lidar_pattern.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief An example of extrinsic parameter calibration
 * NOTE: If the translation of the extrinsic parameter is known and very
 * accurate, you can only optimize the rotation, so that the accuracy of the
 * solved external parameter is higher.
 *
 */

int main(int argc, char **argv) {
  if (argc > 1) {
    std::cerr << argv[0];
    return -1;
  }

  bool detection_success = true;
  bool real_data = false;
  // Camera to Car extrinsic
  if (true) {
    // Camera intrinsic
    double width = 2880;
    double height = 1860;
    std::vector<std::vector<double>> intrinsic = {
      {0.4920860489431918 * width, 0.0 * width, 0.5 * width},
      {0.0 * height, 0.7615706211434203 * height, 0.5 * height},
      {0.0, 0.0, 1.0}
    };

    // Camera distortion coefficients: [k1, k2, p1, p2, k3]
    std::vector<double> dist = {
      -0.29841137985555577,
      0.11993645065259309,
      0.0,
      0.0,
      -0.02381022479995583
    };

    // coordinates in the lidar coordinate system
    std::vector<std::vector<float>> obj_pts;
    for (int i = 0; i<4; i++) {
      // Load file!
      char fname[50];
      sprintf(fname, "output/fiducial%d.pcd", i);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile<pcl::PointXYZ> (fname, *cloud);
      for (auto pt : cloud->points) {
        std::vector<float> asvec = {
          pt.x,
          pt.y,
          pt.z,
        };
        obj_pts.push_back(asvec);
      }
    }
    std::cout << "num lidar points: " << obj_pts.size() << "\n";

    // detected image points
    // order is increasing top to bottom, left to right
    std::vector<std::vector<float>> pts2d;
    std::string cam_path = "output/cam_fiducial_corners.json";
    std::ifstream f( cam_path.c_str() );
    json jason = json::parse(f);
    for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
        pts2d.push_back(jason[std::to_string(i).c_str()][std::to_string(j).c_str()]);
      }
    }

    // calibration result
    std::vector<float> rvec = {0.0, 0.0, 0.0};
    std::vector<float> tvec = {0.0, 0.0, 0.0};
    solveCamPnP(obj_pts, pts2d, intrinsic, dist, rvec, tvec); // solver

    json extrinsics;
    extrinsics["r"] = rvec;
    extrinsics["t"] = tvec;

    std::ofstream extrinsics_file("output/extrinsics.json");
    if (!extrinsics_file.is_open()) {
      std::cerr << "Failed to open extrinsics file\n";
      return -1;
    }

    extrinsics_file << std::setw(4) << extrinsics << std::endl;
    extrinsics_file.close();
  }

  // LiDAR to Car extrinsic
  if (detection_success && real_data) {
    std::vector<std::vector<float>> lidar_pts; // detected lidar points
    std::vector<std::vector<float>>
        obj_pts;                   // coordinates in the car coordinate system
    std::vector<float> rvec, tvec; // calibration result
    solveLidarPnP(obj_pts, lidar_pts, rvec, tvec); // solversolver
  }

  return 0;
}
