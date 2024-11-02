#include <vector>
#include <string>
#include <eigen3/Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <chrono>
#include <string>
#include <random>

#include "logging.h"
#include "robot.h"


#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <iostream>
#include <hpp/fcl/distance.h>

struct DistanceResult {
    double distance;
    Eigen::Vector3d direction;
};

// Function to check distance and compute shortest distance and direction between the cube and capsules
std::vector<DistanceResult> check_distance_and_direction(const pinocchio::GeometryModel& geom_model, pinocchio::GeometryData& geom_data) {
    std::vector<DistanceResult> results;
    pinocchio::computeDistances(geom_model, geom_data);

    for (size_t i = 0; i < geom_data.distanceResults.size(); ++i) {
      LOG_DEBUG << "Checking distance for capsule " << i; 
        const auto& result = geom_data.distanceResults[i];
        if (result.min_distance < std::numeric_limits<double>::infinity()) {
            Eigen::Vector3d direction = result.nearest_points[0] - result.nearest_points[1];
            results.push_back({result.min_distance, direction});
        } else {
            results.push_back({std::numeric_limits<double>::infinity(), Eigen::Vector3d::Zero()});
        }
    }
    return results;
}

// Function to set up the model, geometry, and run the distance computation
void setup_and_run() {
    // Create model and data
    pinocchio::Model model;

    // Create geometry model and data
    pinocchio::GeometryModel geom_model;


    LOG_INFO << "Adding geometry objects...";

    // Add a cube directly (custom shape)
    pinocchio::FrameIndex cube_frame = model.addFrame(
      pinocchio::Frame("cube", 0, 0, pinocchio::SE3::Identity(), pinocchio::BODY));
    std::shared_ptr<hpp::fcl::Box> cube_geometry(new hpp::fcl::Box(1.0, 1.0, 1.0));
    geom_model.addGeometryObject(
      pinocchio::GeometryObject("cube", cube_frame, 0, pinocchio::SE3::Identity(), cube_geometry, "", Eigen::Vector3d::Ones()));

    // Add capsules directly (custom shapes)
    for (int i = 0; i < 5; ++i) {
        std::string capsule_name = "capsule" + std::to_string(i);
        pinocchio::FrameIndex capsule_frame = model.addFrame(
          pinocchio::Frame(capsule_name, 0, 0, pinocchio::SE3::Identity(), pinocchio::BODY));
        std::shared_ptr<hpp::fcl::Capsule> capsule_geometry(new hpp::fcl::Capsule(0.5, 1.0));
        geom_model.addGeometryObject(
          pinocchio::GeometryObject(capsule_name, capsule_frame, 0, pinocchio::SE3::Identity(), capsule_geometry, "", Eigen::Vector3d::Ones()));
    }

    geom_model.addAllCollisionPairs();
    pinocchio::Data data(model);
    pinocchio::GeometryData geom_data(geom_model);

    LOG_DEBUG << "Model number of frames: " << model.nframes;
    LOG_DEBUG << "GeometryModel number of objects: " << geom_model.geometryObjects.size();


    if (!model.check(data)) {
        LOG_ERROR << "Model and data are not consistent!";
        return;
    }


    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);


    // Compute distances and get the shortest distance and direction
    LOG_INFO << "Computing distances...";
    std::vector<DistanceResult> results = check_distance_and_direction(geom_model, geom_data);

    // Print results
    LOG_INFO << "Results:";
    for (size_t i = 0; i < results.size(); ++i) {
        LOG_INFO << "Capsule " << i << " - Distance: " << results[i].distance << " Direction: " << results[i].direction.transpose() << std::endl;
    }
}


int main()
{
  init_logging();

  const std::string TARGET_LINK_NAME = "tool_frame";
  const std::string urdf_path = "../robots/gen3.urdf";
  Robot robot(urdf_path);

  auto random_pose = robot.random_valid_pose(TARGET_LINK_NAME);
  LOG_INFO << "random pose: \n" << random_pose;

  setup_and_run();


}


