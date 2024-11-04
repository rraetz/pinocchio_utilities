#include <vector>
#include <string>
#include <eigen3/Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <chrono>
#include <string>
#include <random>

#include "logging.h"
#include "robot.h"

#include <MeshcatCpp/Material.h>
#include <MeshcatCpp/Meshcat.h>
#include <MeshcatCpp/Shape.h>
#include "meshcat_utilities.h"

#include "pinocchio/multibody/geometry.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
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
      LOG_DEBUG << "Checking distance for cylinder " << i; 
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


enum GeometryType {
    BOX,
    SPHERE,
    CYLINDER,
    MESH
};


const GeometryType detect_geometry_type(const pinocchio::GeometryObject& object) {
    std::shared_ptr<hpp::fcl::CollisionGeometry> geometry = object.geometry;
    if (object.meshPath != "") {
        return GeometryType::MESH;
    } else if (std::dynamic_pointer_cast<hpp::fcl::Box>(geometry)) {
        return GeometryType::BOX;
    } else if (std::dynamic_pointer_cast<hpp::fcl::Sphere>(geometry)) {
        return GeometryType::SPHERE;
    } else if (std::dynamic_pointer_cast<hpp::fcl::Cylinder>(geometry)) {
        return GeometryType::CYLINDER;
    } else {
        throw std::runtime_error("Geometry type not supported");
    }
}


void addBox(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double x, double y, double z) 
{
    // pinocchio::FrameIndex frame = model.addFrame(
    //     pinocchio::Frame(name, 0, 0, pose, pinocchio::BODY)
    // );
    auto geometry = std::make_shared<hpp::fcl::Box>(x, y, z);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
}

void addSphere(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius) 
{
    auto geometry = std::make_shared<hpp::fcl::Sphere>(radius);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
}

void addCylinder(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius, double length) 
{
    auto geometry = std::make_shared<hpp::fcl::Cylinder>(radius, length);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
}



void add_geometry_model_to_meshcat(MeshcatCpp::Meshcat& meshcat, const pinocchio::GeometryModel& geom_model)
{
  static MeshcatCpp::Material m;
  for (const auto& object : geom_model.geometryObjects) 
  {
    const GeometryType geometry_type = detect_geometry_type(object);
    LOG_DEBUG << "Detected geometry type: " << geometry_type;

    const int red = rand() % 255;
    const int green = rand() % 255;
    const int blue = rand() % 255;
    m.set_color(red, green, blue);

    if (geometry_type == GeometryType::BOX)
    {
      auto box = *(std::dynamic_pointer_cast<hpp::fcl::Box>(object.geometry));
      const double x = box.halfSide[0]*2;
      const double y = box.halfSide[1]*2;
      const double z = box.halfSide[2]*2;
      meshcat.set_object(object.name, MeshcatCpp::Box(x, y, z), m);
      LOG_DEBUG << "Box with name " << object.name << " added to Meshcat";
    }

    else if (geometry_type == GeometryType::SPHERE)
    {
      auto sphere = *(std::dynamic_pointer_cast<hpp::fcl::Sphere>(object.geometry));
      const double radius = sphere.radius;
      meshcat.set_object(object.name, MeshcatCpp::Sphere(radius), m);
      LOG_DEBUG << "Sphere with name " << object.name << " added to Meshcat";
    }

    else if (geometry_type == GeometryType::CYLINDER)
    {
      // FIXME: The orientation is not correct. In Meshcat, the cylinder is oriented along the y-axis
      auto cylinder = *(std::dynamic_pointer_cast<hpp::fcl::Cylinder>(object.geometry));
      const double radius = cylinder.radius;
      const double length = cylinder.halfLength*2;
      meshcat.set_object(object.name, MeshcatCpp::Cylinder(radius, length), m);
      LOG_DEBUG << "Cylinder with name " << object.name << " added to Meshcat";
    }
      
    else if (geometry_type == GeometryType::MESH)
    {
      auto mesh_path = object.meshPath;
      if (mesh_path.find(".STL") != std::string::npos) {
        LOG_WARNING << "The mesh file of " << mesh_path << 
          " ending is capitalized. Please use lowercase instead.";
      }
      meshcat.set_object(object.name, MeshcatCpp::Mesh(mesh_path), m);
      LOG_DEBUG << "Mesh with name " << object.name << " added to Meshcat";
    }
  }
}


void update_meshcat_geometry_poses(MeshcatCpp::Meshcat& meshcat, const pinocchio::GeometryModel& geom_model, const pinocchio::GeometryData& geom_data)
{
    for (pinocchio::GeomIndex id=0; id < geom_model.ngeoms; ++id)
    {
        const auto& object = geom_model.geometryObjects[id];
        auto pose = geom_data.oMg[id];
        // LOG_DEBUG << "Updating pose for object " << object.name << " with pose: \n" << pose;
        meshcat.set_transform(object.name, SE3_to_matrix_view(pose));
    }
}
   


int main()
{
    init_logging();


    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;
    MeshcatCpp::Meshcat meshcat;

    const std::string TARGET_LINK_NAME = "tool_frame";
    const std::string urdf_path = "../robots/gen3.urdf";
    Robot robot(urdf_path);


    LOG_INFO << "Adding geometry objects...";
    pinocchio::SE3 cube_pose = pinocchio::SE3::Identity();
    cube_pose.translation() << 0, 0.5, 0.3;
    addBox(geom_model, "cube", cube_pose, 0.2, 0.5, 0.1);
    pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
    sphere_pose.translation() << -0.2, -0.3, 0.5;
    addSphere(geom_model, "sphere", sphere_pose, 0.1);

    // Create data containers for environment
    pinocchio::Data data(model);
    pinocchio::GeometryData geom_data(geom_model);

    // Add geometry objects to meshcat
    add_geometry_model_to_meshcat(meshcat, geom_model);
    add_geometry_model_to_meshcat(meshcat, robot.geom_model_);

    // Update static objects in meshcat
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);
    update_meshcat_geometry_poses(meshcat, geom_model, geom_data);



    for (int i=0; i<50; i++)
    {
        // Exagerated joint positions
        auto q = robot.random_valid_joint_positions()*2;
        robot.update_joint_positions(q);

        pinocchio::updateGeometryPlacements(robot.model_, robot.data_, robot.geom_model_, robot.geom_data_);
        update_meshcat_geometry_poses(meshcat, robot.geom_model_, robot.geom_data_);

        pinocchio::computeCollisions(robot.geom_model_, robot.geom_data_);

        // Print the status of all the collision pairs
        for (size_t k = 0; k < robot.geom_model_.collisionPairs.size(); ++k)
        {
            const pinocchio::CollisionPair & cp = robot.geom_model_.collisionPairs[k];
            const hpp::fcl::CollisionResult & cr = robot.geom_data_.collisionResults[k];

            if (cr.isCollision())
            {
                LOG_INFO << "collision: " << cp.first << " , " << cp.second;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    meshcat.join();


    // // Compute distances and get the shortest distance and direction
    // LOG_INFO << "Computing distances...";
    // std::vector<DistanceResult> results = check_distance_and_direction(geom_model, geom_data);

    // // Print results
    // LOG_INFO << "Results:";
    // for (size_t i = 0; i < results.size(); ++i) {
    //     LOG_INFO << "Capsule " << i << " - Distance: " << results[i].distance << " Direction: " << results[i].direction.transpose() << std::endl;
    // }

}


