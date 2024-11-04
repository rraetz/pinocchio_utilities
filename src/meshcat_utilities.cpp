#include "meshcat_utilities.h"

MeshcatCpp::MatrixView<double> SE3_to_matrix_view(const pinocchio::SE3 &transform) {
  static Eigen::Matrix4d matrix;
  matrix = transform.toHomogeneousMatrix();
  constexpr auto order = MeshcatCpp::MatrixStorageOrdering::ColumnMajor;
  return MeshcatCpp::make_matrix_view(matrix.data(), 4, 4, order);
}
