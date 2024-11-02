#include <MeshcatCpp/Material.h>
#include <MeshcatCpp/Meshcat.h>
#include <MeshcatCpp/Shape.h>
#include <pinocchio/spatial/se3.hpp>



MeshcatCpp::MatrixView<double> SE3_to_matrix_view(const pinocchio::SE3 &transform);
