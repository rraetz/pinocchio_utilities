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




int main()
{
    ///////////////////////////////////////////
    // Setup
    ///////////////////////////////////////////

    init_logging();

    const std::string TARGET_LINK_NAME = "tool_frame";
    const std::string urdf_path = "robots/gen3_simplified.urdf";
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

    // Add 10 random spheres around the origin
    for (int i=0; i<10; i++)
    {
        pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
        sphere_pose.translation() << 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX;
        add_sphere(env_geom_model, "sphere"+std::to_string(i), sphere_pose, 0.1);
    }
    
    // Add also 10 random boxes around the origin
    for (int i=0; i<10; i++)
    {
        pinocchio::SE3 box_pose = pinocchio::SE3::Identity();
        box_pose.translation() << -0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX;
        add_box(env_geom_model, "box"+std::to_string(i), box_pose, 0.1, 0.2, 0.3);
    }

    // Fill data containers for environment
    env_data = pinocchio::Data(env_model);
    env_geom_data = pinocchio::GeometryData(env_geom_model);

    // Populate combined collision model
    pinocchio::appendGeometryModel(combined_geom_model, robot.geom_model_);
    append_collision_pair_mapping(combined_geom_model, robot.geom_model_);
    pinocchio::appendGeometryModel(combined_geom_model, env_geom_model);
    append_collision_pair_mapping(combined_geom_model, env_geom_model);


    // Add collision pairs between each robot link and each object in the environment
    pinocchio::GeomIndex id_env_start = combined_geom_model.ngeoms-env_geom_model.ngeoms;
    for (pinocchio::GeomIndex id_robot=0; id_robot < id_env_start; ++id_robot)
    {
        for (pinocchio::GeomIndex id_env=id_env_start; id_env < combined_geom_model.ngeoms; ++id_env)
        {
            const auto &name1 = combined_geom_model.geometryObjects[id_robot].name;
            const auto &name2 = combined_geom_model.geometryObjects[id_env].name;
            const pinocchio::CollisionPair collision_pair(id_robot, id_env);
            combined_geom_model.addCollisionPair(collision_pair);
            LOG_DEBUG << "Added collision pair between " << name1 << " and " << name2;
        }
    }

    combined_geom_data = pinocchio::GeometryData(combined_geom_model);

    add_geometry_model_to_meshcat(meshcat, combined_geom_model);


    ///////////////////////////////////////////
    // Run the simulation
    ///////////////////////////////////////////
    for (int i=0; i<50; i++)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Update robot joint positions
        auto q = robot.random_valid_joint_positions();
        robot.update_joint_positions(q);
        pinocchio::updateGeometryPlacements(robot.model_, robot.data_, combined_geom_model, combined_geom_data);

        // Collision detection
        pinocchio::computeCollisions(combined_geom_model, combined_geom_data);
        has_collision(combined_geom_model, combined_geom_data);
        compute_distances(combined_geom_model, combined_geom_data);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        LOG_INFO << "Elapsed time: " << elapsed_time.count()*1e-3 << " ms";

        // Display
        update_meshcat_geometry_poses(meshcat, combined_geom_model, combined_geom_data);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    meshcat.join();
}


