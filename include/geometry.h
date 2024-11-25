#pragma once

#include "pinocchio/multibody/geometry.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/geometry.hpp>

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

