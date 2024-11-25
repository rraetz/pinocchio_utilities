#pragma once

#include "pinocchio/multibody/geometry.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/collision/collision.hpp>

void add_box_geometry(
    pinocchio::GeometryModel& geom_model, 
    const std::string& name, 
    const pinocchio::SE3& pose, 
    double x, double y, double z);

void add_sphere_geometry(
    pinocchio::GeometryModel& geom_model, 
    const std::string& name, 
    const pinocchio::SE3& pose, 
    double radius);

void add_cylinder_geometry(
    pinocchio::GeometryModel& geom_model, 
    const std::string& name, 
    const pinocchio::SE3& pose, 
    double radius, double length); 

bool has_collision(
    const pinocchio::GeometryModel& geom_model, 
    const pinocchio::GeometryData& geom_data);

// To be called after pinocchio::appendGeometryModel()
void append_collision_pair_mapping(
    pinocchio::GeometryModel &combined_geom_model, 
    const pinocchio::GeometryModel &source_geom_model);

void compute_distances(const pinocchio::GeometryModel &geom_model, pinocchio::GeometryData &geom_data);

// Add collision pairs between each robot link and each object in the environment to a combined geometry model
void enable_collisions_between_robot_and_environment(
    pinocchio::GeometryModel &combined_geom_model, 
    const pinocchio::GeometryModel &robot_geom_model, 
    const pinocchio::GeometryModel &env_geom_model);


void load_robot_from_urdf(
    const std::string &urdf_path, 
    pinocchio::Model &model, 
    pinocchio::Data &data, 
    pinocchio::GeometryModel &geom_model,
    pinocchio::GeometryData &geom_data,
    bool enable_self_collisions);

