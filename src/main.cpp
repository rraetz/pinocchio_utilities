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

void add_box(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double x, double y, double z) 
{
    auto geometry = std::make_shared<hpp::fcl::Box>(x, y, z);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
}

void add_sphere(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius) 
{
    auto geometry = std::make_shared<hpp::fcl::Sphere>(radius);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
}

void add_cylinder(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius, double length) 
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
        meshcat.set_transform(object.name, SE3_to_matrix_view(pose));
    }
}
   

bool has_collision(const pinocchio::GeometryModel& geom_model, const pinocchio::GeometryData& geom_data)
{
    bool return_value = false;
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
        const pinocchio::CollisionPair & cp = geom_model.collisionPairs[k];
        const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
        if (cr.isCollision())
        {
            return_value = true;
            const auto &name1 = geom_model.geometryObjects[cp.first].name;
            const auto &name2 = geom_model.geometryObjects[cp.second].name;
            LOG_DEBUG << "Collision: " << name1 << " <-> " << name2;
        }
    }
    return return_value;
}

int main()
{
    init_logging();


    pinocchio::Model env_model;
    pinocchio::GeometryModel env_geom_model;
    MeshcatCpp::Meshcat meshcat;

    const std::string TARGET_LINK_NAME = "tool_frame";
    const std::string urdf_path = "../robots/gen3.urdf";
    Robot robot(urdf_path);


    LOG_INFO << "Adding geometry objects...";
    pinocchio::SE3 cube_pose = pinocchio::SE3::Identity();
    cube_pose.translation() << 0, 0.5, 0.3;
    add_box(env_geom_model, "cube", cube_pose, 0.2, 0.5, 0.1);
    pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
    sphere_pose.translation() << -0.2, -0.3, 0.5;
    add_sphere(env_geom_model, "sphere", sphere_pose, 0.1);

    // Create data containers for environment
    pinocchio::Data env_data(env_model);
    pinocchio::GeometryData env_geom_data(env_geom_model);

    // Add geometry objects to meshcat
    add_geometry_model_to_meshcat(meshcat, env_geom_model);
    add_geometry_model_to_meshcat(meshcat, robot.geom_model_);

    // Update static objects in meshcat
    pinocchio::updateGeometryPlacements(env_model, env_data, env_geom_model, env_geom_data);
    update_meshcat_geometry_poses(meshcat, env_geom_model, env_geom_data);

    // Create combined collision model
    pinocchio::GeometryModel geom_model;
    pinocchio::appendGeometryModel(geom_model, robot.geom_model_);
    pinocchio::appendGeometryModel(geom_model, env_geom_model);

    geom_model.collisionPairMapping.conservativeResize(geom_model.ngeoms, geom_model.ngeoms);
    geom_model.collisionPairMapping.fill(-1);
    geom_model.collisionPairMapping.block(0, 0, robot.geom_model_.ngeoms, robot.geom_model_.ngeoms) = robot.geom_model_.collisionPairMapping;
    geom_model.collisionPairMapping.block(robot.geom_model_.ngeoms, robot.geom_model_.ngeoms, env_geom_model.ngeoms, env_geom_model.ngeoms) = env_geom_model.collisionPairMapping;

    // Add collision pairs between each individual robot link and each object in the environment
    pinocchio::GeomIndex id2_start = geom_model.ngeoms-env_geom_model.ngeoms;
    for (pinocchio::GeomIndex id=0; id < id2_start; ++id)
    {
        for (pinocchio::GeomIndex id2=id2_start; id2 < geom_model.ngeoms; ++id2)
        {
            const auto &name1 = geom_model.geometryObjects[id].name;
            const auto &name2 = geom_model.geometryObjects[id2].name;
            LOG_DEBUG << "Adding collision pair between " << name1 << " and " << name2;
            const pinocchio::CollisionPair collision_pair(id, id2);
            geom_model.addCollisionPair(collision_pair);
        }
    }
    pinocchio::GeometryData geom_data(geom_model);


    for (int i=0; i<50; i++)
    {
        auto q = robot.random_valid_joint_positions();
        robot.update_joint_positions(q);

        pinocchio::updateGeometryPlacements(robot.model_, robot.data_, robot.geom_model_, robot.geom_data_);
        update_meshcat_geometry_poses(meshcat, robot.geom_model_, robot.geom_data_);
        pinocchio::updateGeometryPlacements(robot.model_, robot.data_, geom_model, geom_data);


        pinocchio::computeCollisions(geom_model, geom_data);
        has_collision(geom_model, geom_data);


        for (size_t cp_index = 0; cp_index < robot.geom_model_.collisionPairs.size(); ++cp_index)
        {
            const pinocchio::CollisionPair & cp = robot.geom_model_.collisionPairs[cp_index];

            const bool is_pair_active = robot.geom_data_.activeCollisionPairs[cp_index];
            const bool is_first_enabled = !robot.geom_model_.geometryObjects[cp.first].disableCollision;
            const bool is_second_enabled = !robot.geom_model_.geometryObjects[cp.second].disableCollision;

            if (is_pair_active && is_first_enabled && is_second_enabled)
            {
                computeDistance(robot.geom_model_, robot.geom_data_, cp_index);
                const auto &name1 = robot.geom_model_.geometryObjects[cp.first].name;
                const auto &name2 = robot.geom_model_.geometryObjects[cp.second].name;
                double distance = robot.geom_data_.distanceResults[cp_index].min_distance;
                // LOG_DEBUG << "Distance: " << name1 << " <-> " << name2 << ": " << distance;
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


