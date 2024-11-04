#include "robot.h"

#include <exception>
#include <cstdlib> 
#include <ctime>    

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "logging.h"




Robot::Robot(const std::string &urdf_path)
{
  pinocchio::urdf::buildModel(urdf_path, model_);
  data_ = pinocchio::Data(model_);
  LOG_INFO << "Model name: " << model_.name;
  LOG_INFO << "Degrees of freedom: " << model_.nv;

  if (model_.nq != model_.nv)
  {
    LOG_ERROR << "Model configuration space size is not equal to the velocity space size. Avoid continuous joints!";
    exit(1);
  }

  // Log link names
  for (int i = 0; i < model_.nframes; i++)
  {
    const pinocchio::Frame &frame = model_.frames[i];
    if (frame.type == pinocchio::FrameType::BODY)
    {
      LOG_INFO << "Link: " << frame.name;
    }
  }

  // Log joint names
  for (int i = 1; i < model_.njoints; i++) 
  {
    std::string joint_name = model_.names[i];  
    int joint_index = model_.joints[i].idx_q();
    double lower_limit = model_.lowerPositionLimit[joint_index];
    double upper_limit = model_.upperPositionLimit[joint_index];
    LOG_INFO << "Joint: " << joint_name << " [" << lower_limit << ", " << upper_limit << "]";
  }


  // Geometry model & data
  pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_);
  geom_model_.addAllCollisionPairs();
  // Remove collisions of adjacent geometries
  for (pinocchio::GeomIndex id=0; id < geom_model_.ngeoms-1; ++id)
  {
    auto collision_pair = pinocchio::CollisionPair(id, id+1);
    geom_model_.removeCollisionPair(collision_pair);
  }
  geom_data_ = pinocchio::GeometryData(geom_model_);


  // Allocate memory for the Jacobian
  J_ = Eigen::MatrixXd(6, model_.nv);

  // Seed the random number generator
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

const pinocchio::SE3 &Robot::random_valid_pose(const std::string &link_name)
{
  Eigen::VectorXd q = pinocchio::randomConfiguration(model_);
  update_joint_positions(q);
  pose_ = forward_kinematics(link_name);
  return pose_;
}

const Eigen::VectorXd &Robot::random_valid_joint_positions()
{
  q_random_ = pinocchio::randomConfiguration(model_);
  return q_random_;
}

void Robot::update_joint_positions(const Eigen::VectorXd &q)
{
  q_ = q;
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
}

const pinocchio::SE3 &Robot::forward_kinematics(const std::string &link_name)
{
  try
  {
    const pinocchio::FrameIndex frame_id = model_.getFrameId(link_name);
    return data_.oMf[frame_id];
  }
  catch (const std::exception &e)
  {
    std::cerr << "Frame " << link_name << " not found!" << std::endl;
    throw std::runtime_error("Frame not found");
  }
}

const Eigen::MatrixXd &Robot::jacobian(const std::string &link_name)
{
  try
  {
    const pinocchio::FrameIndex frame_id = model_.getFrameId(link_name);
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, J_);
    return J_;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Frame " << link_name << " not found!" << std::endl;
    throw std::runtime_error("Frame not found");
  }
}


const int Robot::dof()
{
  return model_.nv;
}

const Eigen::VectorXd &Robot::lower_joint_position_limits()
{
  return model_.lowerPositionLimit;
}

const Eigen::VectorXd &Robot::upper_joint_position_limits()
{
  return model_.upperPositionLimit;
}

const Eigen::VectorXd &Robot::compute_pose_error(const pinocchio::SE3 &target_pose, const std::string &target_link_name)
{
  pinocchio::SE3 end_effector_pose = forward_kinematics(target_link_name);
  pinocchio::SE3 error_SE3 = target_pose.actInv(end_effector_pose);
  pose_error_ = pinocchio::log6(error_SE3).toVector();
  return pose_error_;
}
