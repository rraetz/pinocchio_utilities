#include "logging.h"
#include "robot.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> 
#include <pybind11/stl.h>


namespace py = pybind11;

PYBIND11_MODULE(rapid_ik, m) {

    init_logging();

    m.doc() = R"pbdoc()pbdoc";

    auto random_valid_pose_python = [](
        Robot &robot, 
        const std::string target_link_name
    ){
        pinocchio::SE3 pose = robot.random_valid_pose(target_link_name);
        return pose.toHomogeneousMatrix();
    };

    auto forward_kinematics_python = [](
        Robot &robot, 
        const std::string link_name
    ){
        pinocchio::SE3 pose = robot.forward_kinematics(link_name);
        return pose.toHomogeneousMatrix();
    };

    py::class_<Robot>(m, "Robot")
        .def(py::init<const std::string &>(), py::arg("urdf_path"))
        .def("update_joint_positions", &Robot::update_joint_positions, py::arg("q"))
        .def("random_valid_pose", random_valid_pose_python)
        .def("forward_kinematics", forward_kinematics_python, py::arg("link_name"))
        .def("jacobian", &Robot::jacobian, py::arg("link_name"))
        .def("dof", &Robot::dof)
        .def("lower_joint_position_limits", &Robot::lower_joint_position_limits)
        .def("upper_joint_position_limits", &Robot::upper_joint_position_limits)
        .def("compute_pose_error", &Robot::compute_pose_error, py::arg("target_pose"), py::arg("target_link_name"));

}