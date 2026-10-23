#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace pinocchio_primitive_models{
  void createBox(const Eigen::Vector3d& size, const double& mass, pinocchio::Model& model);
  void createBox(const Eigen::Vector3d& size, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel);
  void createSphere(const double& radius, const double& mass, pinocchio::Model& model);
  void createSphere(const double& radius, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel);
  // Cylinder length is measured along the local Z axis.
  void createCylinder(const double& radius, const double& length, const double& mass, pinocchio::Model& model);
  void createCylinder(const double& radius, const double& length, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel);
}
