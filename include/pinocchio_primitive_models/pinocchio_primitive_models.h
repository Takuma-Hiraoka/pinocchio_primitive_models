#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace pinocchio_primitive_models{
  void createBox(const Eigen::Vector3d& size, const double& mass, pinocchio::Model& model);
  void createBox(const Eigen::Vector3d& size, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel);
}
