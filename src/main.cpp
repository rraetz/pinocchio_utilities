#include <vector>
#include <string>
#include <eigen3/Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <chrono>
#include <string>
#include <random>

#include "logging.h"
#include "robot.h"
#include "meshcat_utilities.h"
#include "geometry.h"

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


// struct DistanceResult {
//     double distance;
//     Eigen::Vector3d direction;
// };


// TODO: Add directly combined geom model to meshcat

int main()
{
    init_logging();

    const std::string TARGET_LINK_NAME = "tool_frame";
    const std::string urdf_path = "../robots/gen3.urdf";
    Robot robot(urdf_path);

    pinocchio::Model env_model;
    pinocchio::Data env_data;
    pinocchio::GeometryModel env_geom_model;
    pinocchio::GeometryModel combined_geom_model;
    pinocchio::GeometryModel marker_geom_model;
    pinocchio::GeometryData env_geom_data;
    pinocchio::GeometryData combined_geom_data;
    MeshcatCpp::Meshcat meshcat;

    LOG_INFO << "Adding geometry objects...";
    pinocchio::SE3 cube_pose = pinocchio::SE3::Identity();
    cube_pose.translation() << 0, 0.5, 0.3;
    add_box(env_geom_model, "cube", cube_pose, 0.2, 0.5, 0.1);
    pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
    sphere_pose.translation() << -0.2, -0.3, 0.5;
    add_sphere(env_geom_model, "sphere", sphere_pose, 0.1);

    // Fill data containers for environment
    env_data = pinocchio::Data(env_model);
    env_geom_data = pinocchio::GeometryData(env_geom_model);

    // Add geometry objects to meshcat
    add_geometry_model_to_meshcat(meshcat, env_geom_model);
    add_geometry_model_to_meshcat(meshcat, robot.geom_model_);

    // Update static objects in meshcat
    pinocchio::updateGeometryPlacements(env_model, env_data, env_geom_model, env_geom_data);
    update_meshcat_geometry_poses(meshcat, env_geom_model, env_geom_data);

    // Populate combined collision model
    pinocchio::appendGeometryModel(combined_geom_model, robot.geom_model_);
    append_collision_pair_mapping(combined_geom_model, robot.geom_model_);
    pinocchio::appendGeometryModel(combined_geom_model, env_geom_model);
    append_collision_pair_mapping(combined_geom_model, env_geom_model);


    // Add collision pairs between each individual robot link and each object in the environment
    pinocchio::GeomIndex id2_start = combined_geom_model.ngeoms-env_geom_model.ngeoms;
    for (pinocchio::GeomIndex id=0; id < id2_start; ++id)
    {
        for (pinocchio::GeomIndex id2=id2_start; id2 < combined_geom_model.ngeoms; ++id2)
        {
            const auto &name1 = combined_geom_model.geometryObjects[id].name;
            const auto &name2 = combined_geom_model.geometryObjects[id2].name;
            LOG_DEBUG << "Adding collision pair between " << name1 << " and " << name2;
            const pinocchio::CollisionPair collision_pair(id, id2);
            combined_geom_model.addCollisionPair(collision_pair);
        }
    }

    combined_geom_data = pinocchio::GeometryData(combined_geom_model);


    for (int i=0; i<50; i++)
    {
        auto q = robot.random_valid_joint_positions();
        robot.update_joint_positions(q);

        pinocchio::updateGeometryPlacements(robot.model_, robot.data_, robot.geom_model_, robot.geom_data_);
        update_meshcat_geometry_poses(meshcat, robot.geom_model_, robot.geom_data_);
        pinocchio::updateGeometryPlacements(robot.model_, robot.data_, combined_geom_model, combined_geom_data);


        pinocchio::computeCollisions(combined_geom_model, combined_geom_data);
        has_collision(combined_geom_model, combined_geom_data);


        for (size_t cp_index = 0; cp_index < combined_geom_model.collisionPairs.size(); ++cp_index)
        {
            const pinocchio::CollisionPair & cp = combined_geom_model.collisionPairs[cp_index];
            const bool is_pair_active = combined_geom_data.activeCollisionPairs[cp_index];
            const bool is_first_enabled = !combined_geom_model.geometryObjects[cp.first].disableCollision;
            const bool is_second_enabled = !combined_geom_model.geometryObjects[cp.second].disableCollision;

            if (is_pair_active && is_first_enabled && is_second_enabled)
            {
                computeDistance(combined_geom_model, combined_geom_data, cp_index);
                const auto &name1 = combined_geom_model.geometryObjects[cp.first].name;
                const auto &name2 = combined_geom_model.geometryObjects[cp.second].name;
                const auto &result = combined_geom_data.distanceResults[cp_index];
                const double distance = result.min_distance;
                const auto &p1 = result.nearest_points[0];
                const auto &p2 = result.nearest_points[1];
                LOG_DEBUG << "Distance: " << name1 << " <-> " << name2 << ": " << distance;
                LOG_DEBUG << p1;
                LOG_DEBUG << p2;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    meshcat.join();

}


