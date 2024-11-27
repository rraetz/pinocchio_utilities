#include <vector>
#include <string>
#include <chrono>
#include <string>
#include <random>

#include "logging.hpp"
#include "meshcat_utilities.hpp"
#include "pinocchio_utilities.hpp"






int main()
{
    ///////////////////////////////////////////
    // Setup
    ///////////////////////////////////////////

    init_logging();

    // Declare Pinocchio model and data containers
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;
    pinocchio::GeometryModel robot_geom_model;
    pinocchio::GeometryData robot_geom_data;
    pinocchio::GeometryModel env_geom_model;
    pinocchio::GeometryData env_geom_data;
    pinocchio::GeometryModel combined_geom_model;
    pinocchio::GeometryData combined_geom_data;

    // Load robot from URDF
    const std::string urdf_path = "robots/gen3_simplified.urdf";
    load_robot_from_urdf(urdf_path, robot_model, robot_data, robot_geom_model, robot_geom_data, true);

    // Add 10 random spheres around the origin
    for (int i=0; i<10; i++)
    {
        pinocchio::SE3 sphere_pose = pinocchio::SE3::Identity();
        sphere_pose.translation() << 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX, 0.5*rand()/RAND_MAX;
        add_sphere(env_geom_model, "sphere"+std::to_string(i), sphere_pose, 0.1);
    }

    // Fill data container for environment
    env_geom_data = pinocchio::GeometryData(env_geom_model);

    // Populate combined collision model
    pinocchio::appendGeometryModel(combined_geom_model, robot_geom_model);
    append_collision_pair_mapping(combined_geom_model, robot_geom_model);
    pinocchio::appendGeometryModel(combined_geom_model, env_geom_model);
    append_collision_pair_mapping(combined_geom_model, env_geom_model);  

    // Enable collisions
    enable_collisions_between_robot_and_environment(combined_geom_model, robot_geom_model, env_geom_model);
    combined_geom_data = pinocchio::GeometryData(combined_geom_model);

    // Add geometry to meshcat
    MeshcatCpp::Meshcat meshcat;
    add_geometry_model_to_meshcat(meshcat, combined_geom_model);



    ///////////////////////////////////////////
    // Run
    ///////////////////////////////////////////
    for (int i=0; i<50; i++)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        auto q = pinocchio::randomConfiguration(robot_model) * 2.0; // To also check self-collisions
        pinocchio::forwardKinematics(robot_model, robot_data, q);
        pinocchio::updateGeometryPlacements(robot_model, robot_data, combined_geom_model, combined_geom_data);

        // pinocchio::computeCollisions(combined_geom_model, combined_geom_data);
        compute_collisions(combined_geom_model, combined_geom_data);
        has_collision(combined_geom_model, combined_geom_data);
        // // compute_distances(combined_geom_model, combined_geom_data);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        LOG_DEBUG << "Elapsed time: " << elapsed_time.count()*1e-3 << " ms";

        update_meshcat_geometry_poses(meshcat, combined_geom_model, combined_geom_data);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    meshcat.join();
}


