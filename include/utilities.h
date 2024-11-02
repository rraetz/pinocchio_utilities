#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

std::vector<double> eigen_to_double_vector(Eigen::VectorXd &v)
{
  return std::vector<double>(&v[0], v.data()+v.size());
}

const std::vector<double> eigen_to_double_vector(const Eigen::VectorXd &v)
{
  return std::vector<double>(&v[0], v.data()+v.size());
}