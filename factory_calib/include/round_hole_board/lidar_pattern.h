#include <eigen3/Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace lidarcalib {

class LidarDetector {

public:
  LidarDetector(){};
  ~LidarDetector(){};

  void LidarDetection(std::string pcds_dir, json cfg);
};

} // lidarcalib
