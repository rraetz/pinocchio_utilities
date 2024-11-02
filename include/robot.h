#pragma once

#include <string>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>


class Robot
{
public:
  // Constructor that loads the URDF model
  Robot(const std::string &urdf_path);

  // Function to get a random valid pose for a given link name
  const pinocchio::SE3 &random_valid_pose(const std::string &link_name);

  // Function to get a random valid joint position
  const Eigen::VectorXd &random_valid_joint_positions();

  // Updates the internal state of the robot (required to retrieve any information)
  void update_joint_positions(const Eigen::VectorXd &q);

  // Forward kinematics to compute the pose of a link
  const pinocchio::SE3 &forward_kinematics(const std::string &link_name);

  // Jacobian computation
  const Eigen::MatrixXd &jacobian(const std::string &link_name);

  // Return the number of degrees of freedom
  const int dof();

  // Return the lower joint position limits
  const Eigen::VectorXd &lower_joint_position_limits();

  // Return the upper joint position limits
  const Eigen::VectorXd &upper_joint_position_limits();

  // Computes the pose error between current pose (updated with update_joint_positions) and a target pose
  const Eigen::VectorXd &compute_pose_error(const pinocchio::SE3 &target_pose, const std::string &target_link_name);

  pinocchio::Model model_;
  pinocchio::Data data_;


private:
  Eigen::VectorXd q_;
  Eigen::VectorXd q_random_;
  Eigen::MatrixXd J_;
  pinocchio::SE3 pose_;
  Eigen::VectorXd pose_error_;
};