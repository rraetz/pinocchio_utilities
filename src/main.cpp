#include <vector>
#include <string>
#include <chrono>
#include <string>
#include <random>

#include "logging.h"
#include "robot.h"
#include "meshcat_utilities.h"
#include "geometry.h"

#include <eigen3/Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/multibody/geometry.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/collision/collision.hpp>



int main()
{
    ///////////////////////////////////////////
    // Setup
    ///////////////////////////////////////////

    init_logging();

    const std::string urdf_path = "robots/gen3_simplified.urdf";

    MeshcatCpp::Meshcat meshcat;
    Robot robot(urdf_path);

    pinocchio::Model env_model;
    pinocchio::Data env_data;
    pinocchio::GeometryModel env_geom_model;
    pinocchio::GeometryData env_geom_data;
    pinocchio::GeometryModel combined_geom_model;
    pinocchio::GeometryData combined_geom_data;


    pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
    sphere_pose.translation() << -0.2, -0.3, 0.5;
    add_sphere_geometry(env_geom_model, "sphere", sphere_pose, 0.1);

    // Add 10 random spheres around the origin
    for (int i=0; i<10; i++)
    {
        pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
        sphere_pose.translation() << 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX;
        add_sphere_geometry(env_geom_model, "sphere"+std::to_string(i), sphere_pose, 0.1);
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

        pinocchio::computeCollisions(combined_geom_model, combined_geom_data);
        has_collision(combined_geom_model, combined_geom_data);
        compute_distances(combined_geom_model, combined_geom_data);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        LOG_INFO << "Elapsed time: " << elapsed_time.count()*1e-3 << " ms";

        update_meshcat_geometry_poses(meshcat, combined_geom_model, combined_geom_data);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    meshcat.join();
}


