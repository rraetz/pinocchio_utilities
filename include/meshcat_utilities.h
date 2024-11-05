#pragma once

#include <MeshcatCpp/Material.h>
#include <MeshcatCpp/Meshcat.h>
#include <MeshcatCpp/Shape.h>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/multibody/geometry.hpp"


MeshcatCpp::MatrixView<double> SE3_to_matrix_view(const pinocchio::SE3 &transform);

void add_geometry_model_to_meshcat(MeshcatCpp::Meshcat& meshcat, const pinocchio::GeometryModel& geom_model);

void update_meshcat_geometry_poses(MeshcatCpp::Meshcat& meshcat, const pinocchio::GeometryModel& geom_model, const pinocchio::GeometryData& geom_data);

   