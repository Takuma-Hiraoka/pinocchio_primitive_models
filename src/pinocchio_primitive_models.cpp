#include "pinocchio_primitive_models/pinocchio_primitive_models.h"
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <coal/shape/geometric_shapes.h>
#include <memory>

namespace pinocchio_primitive_models{
  void createBox(const Eigen::Vector3d& size, const double& mass, pinocchio::Model& model) {
    pinocchio::GeometryModel cmodel;
    createBox(size, mass, model, cmodel);
  }

  void createBox(const Eigen::Vector3d& size, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel) {

    pinocchio::JointIndex jointId = model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "base_joint");
    pinocchio::Inertia inertia = pinocchio::Inertia::FromBox(mass, size[0], size[1], size[2]);
    model.appendBodyToJoint(jointId, inertia, pinocchio::SE3::Identity());

    std::shared_ptr<coal::Box> box = std::make_shared<coal::Box>(size[0], size[1], size[2]);
    pinocchio::GeometryObject geom("box", jointId, pinocchio::SE3::Identity(), box);
    cmodel.addGeometryObject(geom, model);
  }
}
