#pragma once

#include "logging.hpp"

#include "pinocchio/multibody/geometry.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/parsers/urdf.hpp>

void add_box(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double x, double y, double z) 
{
    auto geometry = std::make_shared<hpp::fcl::Box>(x, y, z);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
    LOG_INFO << "Geometry BOX added: " << name;
}


void add_sphere(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius) 
{
    auto geometry = std::make_shared<hpp::fcl::Sphere>(radius);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
    LOG_INFO << "Geometry SPHERE added: " << name;
}


void add_cylinder(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius, double length) 
{
    auto geometry = std::make_shared<hpp::fcl::Cylinder>(radius, length);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
    LOG_INFO << "Geometry CYLINDER added: " << name;
}


bool has_collision(const pinocchio::GeometryModel& geom_model, const pinocchio::GeometryData& geom_data)
{
    LOG_DEBUG << "Checking for collisions...";
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

// To be called after pinocchio::appendGeometryModel()
void append_collision_pair_mapping(pinocchio::GeometryModel &combined_geom_model, const pinocchio::GeometryModel &source_geom_model)
{
    // Assumes that the new geometry models are already merged!!!
    const auto new_size = combined_geom_model.ngeoms; 
    const auto source_size = source_geom_model.ngeoms;
    const auto old_size = new_size - source_size;
    const auto n_old_collision_pairs = combined_geom_model.collisionPairs.size() - source_geom_model.collisionPairs.size();

    combined_geom_model.collisionPairMapping.conservativeResize(new_size, new_size);
    combined_geom_model.collisionPairMapping.bottomLeftCorner(source_size, old_size).fill(-1);
    combined_geom_model.collisionPairMapping.topRightCorner(old_size, source_size).fill(-1);
    combined_geom_model.collisionPairMapping.bottomRightCorner(source_size, source_size).fill(-1);

    // Map new collision pairs from the source model to the new combined model
    for (size_t pair_index = 0; pair_index < source_geom_model.collisionPairs.size(); ++pair_index)
    {
        const auto &pair = source_geom_model.collisionPairs[pair_index];
        const auto first = pair.first + old_size;
        const auto second = pair.second + old_size;
        combined_geom_model.collisionPairMapping(first, second) = pair_index + n_old_collision_pairs;
        combined_geom_model.collisionPairMapping(second, first) = pair_index + n_old_collision_pairs;
    }
}


void compute_distances(const pinocchio::GeometryModel &geom_model, pinocchio::GeometryData &geom_data)
{
    pinocchio::computeDistances(geom_model, geom_data);
    for (size_t cp_index = 0; cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
        const auto &result = geom_data.distanceResults[cp_index];
        const pinocchio::CollisionPair & cp = geom_model.collisionPairs[cp_index];
        const auto &name1 = geom_model.geometryObjects[cp.first].name;
        const auto &name2 = geom_model.geometryObjects[cp.second].name;
        const double distance = result.min_distance;
        const auto &p1 = result.nearest_points[0];
        const auto &p2 = result.nearest_points[1];
        const auto position_delta = p1-p2;
        LOG_DEBUG << "Distance: " << name1 << " <-> " << name2 << ": " << distance;
        LOG_DEBUG << "Position delta: " << position_delta;
    }
}


void enable_collisions_between_robot_and_environment(
    pinocchio::GeometryModel &combined_geom_model, 
    const pinocchio::GeometryModel &robot_geom_model, 
    const pinocchio::GeometryModel &env_geom_model)
{
    LOG_INFO << "Enabling collisions between robot and environment...";
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
            LOG_INFO << "Added collision pair between " << name1 << " and " << name2;
        }
    }
}


void load_robot_from_urdf(
    const std::string &urdf_path, 
    pinocchio::Model &model, 
    pinocchio::Data &data, 
    pinocchio::GeometryModel &geom_model,
    pinocchio::GeometryData &geom_data,
    bool enable_self_collisions)
{
    pinocchio::urdf::buildModel(urdf_path, model);
    data = pinocchio::Data(model);

    LOG_INFO << "Model name: " << model.name;
    LOG_INFO << "Degrees of freedom: " << model.nv;

    if (model.nq != model.nv)
    {
    LOG_ERROR << "Model configuration space size is not equal to the velocity space size. Avoid continuous joints!";
    exit(1);
    }

    // Log link names
    for (int i = 0; i < model.nframes; i++)
    {
    const pinocchio::Frame &frame = model.frames[i];
    if (frame.type == pinocchio::FrameType::BODY)
    {
        LOG_INFO << "Link: " << frame.name;
    }
    }

    // Log joint names
    for (int i = 1; i < model.njoints; i++) 
    {
    std::string joint_name = model.names[i];  
    int joint_index = model.joints[i].idx_q();
    double lower_limit = model.lowerPositionLimit[joint_index];
    double upper_limit = model.upperPositionLimit[joint_index];
    LOG_INFO << "Limits joint: " << joint_name << " [" << lower_limit << ", " << upper_limit << "]";
    }

    pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, geom_model);

    if (enable_self_collisions)
    {
        LOG_INFO << "Adding non-adjacent robot self-collisions...";
        geom_model.addAllCollisionPairs();
        // Remove collisions of adjacent geometries
        for (pinocchio::GeomIndex id=0; id < geom_model.ngeoms-1; ++id)
        {
            auto collision_pair = pinocchio::CollisionPair(id, id+1);
            geom_model.removeCollisionPair(collision_pair);
        }
    }
    else
    {
        LOG_INFO << "Adding no robot self-collisions...";
    }
    geom_data = pinocchio::GeometryData(geom_model);
}