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
    // The reduced GeometryObject constructor supplies parentJoint, not a
    // parentFrame. Do not use the overload which resolves parentJoint via it.
    cmodel.addGeometryObject(geom);
  }

  void createSphere(const double& radius, const double& mass, pinocchio::Model& model) {
    pinocchio::GeometryModel cmodel;
    createSphere(radius, mass, model, cmodel);
  }

  void createSphere(const double& radius, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel) {
    pinocchio::JointIndex jointId = model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "base_joint");
    pinocchio::Inertia inertia = pinocchio::Inertia::FromSphere(mass, radius);
    model.appendBodyToJoint(jointId, inertia, pinocchio::SE3::Identity());

    std::shared_ptr<coal::Sphere> sphere = std::make_shared<coal::Sphere>(radius);
    pinocchio::GeometryObject geom("sphere", jointId, pinocchio::SE3::Identity(), sphere);
    cmodel.addGeometryObject(geom);
  }

  void createCylinder(const double& radius, const double& length, const double& mass, pinocchio::Model& model) {
    pinocchio::GeometryModel cmodel;
    createCylinder(radius, length, mass, model, cmodel);
  }

  void createCylinder(const double& radius, const double& length, const double& mass, pinocchio::Model& model, pinocchio::GeometryModel& cmodel) {
    pinocchio::JointIndex jointId = model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "base_joint");
    pinocchio::Inertia inertia = pinocchio::Inertia::FromCylinder(mass, radius, length);
    model.appendBodyToJoint(jointId, inertia, pinocchio::SE3::Identity());

    std::shared_ptr<coal::Cylinder> cylinder = std::make_shared<coal::Cylinder>(radius, length);
    pinocchio::GeometryObject geom("cylinder", jointId, pinocchio::SE3::Identity(), cylinder);
    cmodel.addGeometryObject(geom);
  }
}
