#include <iostream>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
<<<<<<< HEAD
#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_actuator.h"
=======
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
>>>>>>> intial
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using std::make_unique;

namespace drake {
namespace pydrake {

<<<<<<< HEAD

=======
>>>>>>> intial
PYBIND11_MODULE(rigid_body_tree, m) {
  m.doc() = "Bindings for the RigidBodyTree class";

  using drake::multibody::joints::FloatingBaseType;
  using drake::parsers::PackageMap;
  namespace sdf = drake::parsers::sdf;
  using std::shared_ptr;

<<<<<<< HEAD
  py::module::import("pydrake.multibody.collision");
  py::module::import("pydrake.multibody.joints");
  py::module::import("pydrake.multibody.parsers");
  py::module::import("pydrake.multibody.rigid_body");
=======
  py::module::import("pydrake.multibody.parsers");
>>>>>>> intial
  py::module::import("pydrake.multibody.shapes");
  py::module::import("pydrake.util.eigen_geometry");

  py::enum_<FloatingBaseType>(m, "FloatingBaseType")
    .value("kFixed", FloatingBaseType::kFixed)
    .value("kRollPitchYaw", FloatingBaseType::kRollPitchYaw)
    .value("kQuaternion", FloatingBaseType::kQuaternion);

  // TODO(eric.cousineau): Try to decouple these APIs so that `rigid_body_tree`
  // and `parsers` do not form a dependency cycle.
<<<<<<< HEAD
  py::class_<RigidBodyTree<double>> tree_cls(m, "RigidBodyTree");
  tree_cls
=======
  py::class_<RigidBodyTree<double>>(m, "RigidBodyTree")
>>>>>>> intial
    .def(py::init<>())
    .def(py::init(
         [](const std::string& urdf_filename,
            const PackageMap& pmap,
            FloatingBaseType floating_base_type
            ) {
          auto instance = make_unique<RigidBodyTree<double>>();
          drake::parsers::urdf::
            AddModelInstanceFromUrdfFileSearchingInRosPackages(
            urdf_filename,
            pmap,
            floating_base_type,
            nullptr,
            instance.get());
          return instance;
        }),
        py::arg("urdf_filename"),
        py::arg("package_map"),
        py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw)
    .def(py::init(
         [](const std::string& urdf_filename,
            FloatingBaseType floating_base_type
            ) {
          auto instance = make_unique<RigidBodyTree<double>>();
          drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf_filename, floating_base_type, instance.get());
          return instance;
        }),
        py::arg("urdf_filename"),
        py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw)
    .def(py::init(
         [](const std::string& urdf_filename,
            const std::string& joint_type) {
            // FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2
            FloatingBaseType floating_base_type;
            std::cerr << "WARNING: passing joint_type as a string is "
              << "deprecated. Please pass a FloatingBaseType value such as "
              << "FloatingBaseType.kRollPitchYaw" << std::endl;
            if (joint_type == "FIXED") {
              floating_base_type = FloatingBaseType::kFixed;
            } else if (joint_type == "ROLLPITCHYAW") {
              floating_base_type = FloatingBaseType::kRollPitchYaw;
            } else if (joint_type == "QUATERNION") {
              floating_base_type = FloatingBaseType::kQuaternion;
            } else {
              throw(std::invalid_argument("Joint type not supported"));
            }
            auto instance = make_unique<RigidBodyTree<double>>();
            drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                urdf_filename, floating_base_type, instance.get());
            return instance;
        }),
        py::arg("urdf_filename"), py::arg("joint_type") = "ROLLPITCHYAW"
      )
    .def("compile", &RigidBodyTree<double>::compile)
<<<<<<< HEAD
    .def("initialized", &RigidBodyTree<double>::initialized)
    .def("drawKinematicTree", &RigidBodyTree<double>::drawKinematicTree)
=======
>>>>>>> intial
    .def("getRandomConfiguration", [](const RigidBodyTree<double>& tree) {
      std::default_random_engine generator(std::random_device {}());
      return tree.getRandomConfiguration(generator);
    })
    .def("getZeroConfiguration", &RigidBodyTree<double>::getZeroConfiguration)
<<<<<<< HEAD
=======
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v) {
      return tree.doKinematics(q, v);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const VectorX<AutoDiffXd>& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const VectorX<AutoDiffXd>& q,
                            const VectorX<AutoDiffXd>& v) {
      return tree.doKinematics(q, v);
    })
>>>>>>> intial
    .def("CalcBodyPoseInWorldFrame", [](const RigidBodyTree<double>& tree,
                                        const KinematicsCache<double> &cache,
                                        const RigidBody<double> &body) {
      return tree.CalcBodyPoseInWorldFrame(cache, body).matrix();
    })
<<<<<<< HEAD
    .def("get_num_bodies", &RigidBodyTree<double>::get_num_bodies)
    .def("get_num_frames", &RigidBodyTree<double>::get_num_frames)
    .def("get_num_actuators", &RigidBodyTree<double>::get_num_actuators)
    .def("get_num_model_instances",
         &RigidBodyTree<double>::get_num_model_instances)
=======
    .def("centerOfMass", &RigidBodyTree<double>::centerOfMass<double>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set)
    .def("centerOfMassJacobian",
         &RigidBodyTree<double>::centerOfMassJacobian<double>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set,
         py::arg("in_terms_of_qdot") = false)
    .def("get_num_bodies", &RigidBodyTree<double>::get_num_bodies)
    .def("get_num_frames", &RigidBodyTree<double>::get_num_frames)
    .def("get_num_actuators", &RigidBodyTree<double>::get_num_actuators)
>>>>>>> intial
    .def("getBodyOrFrameName",
         &RigidBodyTree<double>::getBodyOrFrameName,
         py::arg("body_or_frame_id"))
    .def("number_of_positions", &RigidBodyTree<double>::get_num_positions)
    .def("get_num_positions", &RigidBodyTree<double>::get_num_positions)
    .def("number_of_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_num_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_body", &RigidBodyTree<double>::get_body,
         py::return_value_policy::reference)
    .def("get_position_name", &RigidBodyTree<double>::get_position_name)
<<<<<<< HEAD
    .def("add_rigid_body", &RigidBodyTree<double>::add_rigid_body)
    .def("addCollisionElement", &RigidBodyTree<double>::addCollisionElement)
    .def("AddCollisionFilterGroupMember",
         &RigidBodyTree<double>::AddCollisionFilterGroupMember,
         py::arg("group_name"), py::arg("body_name"),
         py::arg("model_id"))
    .def("DefineCollisionFilterGroup",
         &RigidBodyTree<double>::DefineCollisionFilterGroup,
         py::arg("name"))
    .def("FindCollisionElement",
         &RigidBodyTree<double>::FindCollisionElement,
         py::arg("id"), py::return_value_policy::reference)
=======
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<double>& cache,
                               const Eigen::Matrix<double, 3,
                                                   Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points,
                                  from_body_or_frame_ind, to_body_or_frame_ind);
    })
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<AutoDiffXd>& cache,
                               const Eigen::Matrix<double, 3,
                                                   Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points,
                                  from_body_or_frame_ind, to_body_or_frame_ind);
    })
    .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                  const KinematicsCache<double>& cache,
                                  int base_or_frame_ind,
                                  int body_or_frame_ind) {
      return tree.relativeTransform(cache, base_or_frame_ind,
        body_or_frame_ind).matrix();
    })
    .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                  const KinematicsCache<AutoDiffXd>& cache,
                                  int base_or_frame_ind,
                                  int body_or_frame_ind) {
      return tree.relativeTransform(cache, base_or_frame_ind,
        body_or_frame_ind).matrix();
    })
>>>>>>> intial
    .def("addFrame", &RigidBodyTree<double>::addFrame, py::arg("frame"))
    .def("FindBody", [](const RigidBodyTree<double>& self,
                        const std::string& body_name,
                        const std::string& model_name = "",
                        int model_id = -1) {
      return self.FindBody(body_name, model_name, model_id);
    }, py::arg("body_name"),
       py::arg("model_name") = "",
       py::arg("model_id") = -1,
       py::return_value_policy::reference)
<<<<<<< HEAD
    .def("FindBodyIndex",
         &RigidBodyTree<double>::FindBodyIndex,
         py::arg("body_name"), py::arg("model_id") = -1)
    .def("FindChildBodyOfJoint", [](const RigidBodyTree<double>& self,
                        const std::string& joint_name,
                        int model_id) {
      return self.FindChildBodyOfJoint(joint_name, model_id);
    }, py::arg("joint_name"), py::arg("model_id") = -1,
       py::return_value_policy::reference)
    .def("FindIndexOfChildBodyOfJoint",
         &RigidBodyTree<double>::FindIndexOfChildBodyOfJoint,
         py::arg("joint_name"), py::arg("model_id") = -1)
=======
>>>>>>> intial
    .def("world",
         static_cast<RigidBody<double>& (RigidBodyTree<double>::*)()>(
             &RigidBodyTree<double>::world),
         py::return_value_policy::reference)
    .def("findFrame", &RigidBodyTree<double>::findFrame,
         py::arg("frame_name"), py::arg("model_id") = -1)
    .def("getTerrainContactPoints",
         [](const RigidBodyTree<double>& self,
            const RigidBody<double>& body,
            const std::string& group_name = "") {
          auto pts = Eigen::Matrix3Xd(3, 0);
          self.getTerrainContactPoints(body, &pts, group_name);
          return pts;
        }, py::arg("body"), py::arg("group_name")="")
<<<<<<< HEAD
    .def_readonly("B", &RigidBodyTree<double>::B)
    .def_readonly("joint_limit_min", &RigidBodyTree<double>::joint_limit_min)
    .def_readonly("joint_limit_max", &RigidBodyTree<double>::joint_limit_max)
    // N.B. This will return *copies* of the actuators.
    // N.B. `def_readonly` implicitly adds `reference_internal` to the getter,
    // which is necessary since an actuator references a `RigidBody` that is
    // most likely owned by this tree.
    .def_readonly("actuators", &RigidBodyTree<double>::actuators)
    .def("GetActuator", &RigidBodyTree<double>::GetActuator,
         py_reference_internal)
    .def("FindBaseBodies", &RigidBodyTree<double>::FindBaseBodies,
         py::arg("model_instance_id") = -1)
    .def("addDistanceConstraint",
         &RigidBodyTree<double>::addDistanceConstraint,
         py::arg("bodyA_index_in"),
         py::arg("r_AP_in"),
         py::arg("bodyB_index_in"),
         py::arg("r_BQ_in"),
         py::arg("distance_in"))
    .def("getNumPositionConstraints",
         &RigidBodyTree<double>::getNumPositionConstraints)
    .def("Clone", &RigidBodyTree<double>::Clone)
    .def("__copy__", &RigidBodyTree<double>::Clone);

  // This lambda defines RigidBodyTree methods which are defined for a given
  // templated type. The methods are either (a) direct explicit template
  // instantiations defined in `rigid_body_tree.cc` or (b) defined in
  // `rigid_body_tree.h` and dependent upon methods of type (a).
  // Methods of type (a) follow the same order as the explicit instantiations in
  // `rigid_body_tree.cc`; if the method is not yet bound, the name has a
  // comment as a placeholder.
  // Methods of type (b) are declared below methods of type (a).
  auto add_rigid_body_tree_typed_methods = [m, &tree_cls](auto dummy) {
    // N.B. The header files use `Scalar` as the scalar-type template
    // parameter, but `T` is used here for brevity.
    using T = decltype(dummy);
    // Type (a) methods:
    tree_cls
      .def("massMatrix", &RigidBodyTree<double>::massMatrix<T>)
      .def("centerOfMass", &RigidBodyTree<double>::centerOfMass<T>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set)
      .def("transformVelocityToQDot", [](const RigidBodyTree<double>& tree,
                                          const KinematicsCache<T>& cache,
                                          const VectorX<T>& v) {
             return tree.transformVelocityToQDot(cache, v);
           })
      .def("transformQDotToVelocity", [](const RigidBodyTree<double>& tree,
                                          const KinematicsCache<T>& cache,
                                          const VectorX<T>& qdot) {
             return tree.transformQDotToVelocity(cache, qdot);
           })
      .def("GetVelocityToQDotMapping", [](const RigidBodyTree<double>& tree,
                                          const KinematicsCache<T>& cache) {
             return tree.GetVelocityToQDotMapping(cache);
           })
      .def("GetQDotToVelocityMapping", [](const RigidBodyTree<double>& tree,
                                          const KinematicsCache<T>& cache) {
             return tree.GetQDotToVelocityMapping(cache);
           })
      .def("dynamicsBiasTerm", &RigidBodyTree<double>::dynamicsBiasTerm<T>,
           py::arg("cache"), py::arg("external_wrenches"),
           py::arg("include_velocity_terms") = true)
      .def("geometricJacobian",
           [](const RigidBodyTree<double>& tree,
              const KinematicsCache<T>& cache, int base_body_or_frame_ind,
              int end_effector_body_or_frame_ind,
              int expressed_in_body_or_frame_ind, bool in_terms_of_qdot) {
             std::vector<int> v_indices;
             auto J = tree.geometricJacobian(
                 cache, base_body_or_frame_ind, end_effector_body_or_frame_ind,
                 expressed_in_body_or_frame_ind, in_terms_of_qdot, &v_indices);
             return py::make_tuple(J, v_indices);
           },
           py::arg("cache"), py::arg("base_body_or_frame_ind"),
           py::arg("end_effector_body_or_frame_ind"),
           py::arg("expressed_in_body_or_frame_ind"),
           py::arg("in_terms_of_qdot") = false)
      .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                    const KinematicsCache<T>& cache,
                                    int base_or_frame_ind,
                                    int body_or_frame_ind) {
          return tree.relativeTransform(cache, base_or_frame_ind,
            body_or_frame_ind).matrix();
        },
        py::arg("cache"),
        py::arg("base_or_frame_ind"), py::arg("body_or_frame_ind"))
      .def("centerOfMassJacobian",
           &RigidBodyTree<double>::centerOfMassJacobian<T>,
           py::arg("cache"),
           py::arg("model_instance_id_set") =
             RigidBodyTreeConstants::default_model_instance_id_set,
           py::arg("in_terms_of_qdot") = false)
      // centroidalMomentumMatrix
      // forwardKinPositionGradient
      .def("geometricJacobianDotTimesV",
           &RigidBodyTree<double>::geometricJacobianDotTimesV<T>,
           py::arg("cache"),
           py::arg("base_body_or_frame_ind"),
           py::arg("end_effector_body_or_frame_ind"),
           py::arg("expressed_in_body_or_frame_ind"))
      .def("centerOfMassJacobianDotTimesV",
           &RigidBodyTree<double>::centerOfMassJacobianDotTimesV<T>,
           py::arg("cache"),
           py::arg("model_instance_id_set") =
             RigidBodyTreeConstants::default_model_instance_id_set)
      // centroidalMomentumMatrixDotTimesV
      .def("positionConstraints",
           &RigidBodyTree<double>::positionConstraints<T>,
           py::arg("cache"))
      .def("positionConstraintsJacobian",
           &RigidBodyTree<double>::positionConstraintsJacobian<T>,
           py::arg("cache"),
           py::arg("in_terms_of_qdot") = true)
      .def("positionConstraintsJacDotTimesV",
           &RigidBodyTree<double>::positionConstraintsJacDotTimesV<T>,
           py::arg("cache"))
      // jointLimitConstriants
      .def("relativeTwist",
           &RigidBodyTree<double>::relativeTwist<T>,
           py::arg("cache"),
           py::arg("base_or_frame_ind"),
           py::arg("body_or_frame_ind"),
           py::arg("expressed_in_body_or_frame_ind"))
      // worldMomentumMatrix
      // worldMomentumMatrixDotTimesV
      // transformSpatialAcceleration
      .def("frictionTorques",
           [](const RigidBodyTree<double>* self, const VectorX<T>& v) {
             return self->frictionTorques(v);
           })
      .def("inverseDynamics", &RigidBodyTree<double>::inverseDynamics<T>,
           py::arg("cache"),
           py::arg("external_wrenches"),
           py::arg("vd"),
           py::arg("include_velocity_terms") = true)
      // resolveCenterOfPressure
      .def("transformVelocityMappingToQDotMapping",
           [](const RigidBodyTree<double>& tree,
              const KinematicsCache<T>& cache,
              const MatrixX<T>& Av) {
             return tree.transformVelocityMappingToQDotMapping(cache, Av);
           })
      .def("transformQDotMappingToVelocityMapping",
           [](const RigidBodyTree<double>& tree,
              const KinematicsCache<T>& cache,
              const MatrixX<T>& Ap) {
             return tree.transformQDotMappingToVelocityMapping(cache, Ap);
           })
      // relativeQuaternionJacobian
      // relativeRollPitchYawJacobian
      // relativeRollPitchYawJacobianDotTimesV
      // relativeQuaternionJacobianDotTimesV
      // CheckCacheValidity
      .def("doKinematics", [](const RigidBodyTree<double>& tree,
                              const VectorX<T>& q) {
        return tree.doKinematics(q);
      })
      .def("doKinematics", [](const RigidBodyTree<double>& tree,
                              const VectorX<T>& q,
                              const VectorX<T>& v) {
        return tree.doKinematics(q, v);
      });
      // CreateKinematicsCacheWithType
      // ComputeMaximumDepthCollisionPoints
    // Type (b) methods:
    tree_cls
      .def("transformPoints", [](const RigidBodyTree<double>& tree,
                                 const KinematicsCache<T>& cache,
                                 const Eigen::Matrix<double, 3,
                                                     Eigen::Dynamic>& points,
                                 int from_body_or_frame_ind,
                                 int to_body_or_frame_ind) {
        return tree.transformPoints(
            cache, points, from_body_or_frame_ind, to_body_or_frame_ind);
      })
      .def("transformPointsJacobian",
           [](const RigidBodyTree<double>& tree,
              const KinematicsCache<T>& cache,
              const Matrix3X<double>& points,
              int from_body_or_frame_ind,
              int to_body_or_frame_ind,
              bool in_terms_of_qdot) {
             return tree.transformPointsJacobian(cache, points,
                  from_body_or_frame_ind, to_body_or_frame_ind,
                  in_terms_of_qdot);
           },
           py::arg("cache"), py::arg("points"),
           py::arg("from_body_or_frame_ind"),
           py::arg("to_body_or_frame_ind"),
           py::arg("in_terms_of_qdot"))
      .def("transformPointsJacobianDotTimesV",
           [](const RigidBodyTree<double>& tree,
              const KinematicsCache<T>& cache,
              const Matrix3X<double>& points,
              int from_body_or_frame_ind,
              int to_body_or_frame_ind) {
             return tree.transformPointsJacobianDotTimesV(cache, points,
                  from_body_or_frame_ind, to_body_or_frame_ind);
           },
           py::arg("cache"), py::arg("points"),
           py::arg("from_body_or_frame_ind"),
           py::arg("to_body_or_frame_ind"));
  };
  // Bind for double and AutoDiff.
  type_visit(
      add_rigid_body_tree_typed_methods, type_pack<double, AutoDiffXd>{});
=======
    .def("massMatrix", &RigidBodyTree<double>::massMatrix<double>)
    .def("dynamicsBiasTerm", &RigidBodyTree<double>::dynamicsBiasTerm<double>,
         py::arg("cache"), py::arg("external_wrenches"),
         py::arg("include_velocity_terms") = true)
    .def("inverseDynamics", &RigidBodyTree<double>::inverseDynamics<double>,
         py::arg("cache"),
         py::arg("external_wrenches"),
         py::arg("vd"),
         py::arg("include_velocity_terms") = true)
    .def("frictionTorques",
         [](const RigidBodyTree<double>* self, const VectorX<double>& v) {
           return self->frictionTorques(v);
         })
    .def_readonly("B", &RigidBodyTree<double>::B)
    .def_readonly("joint_limit_min", &RigidBodyTree<double>::joint_limit_min)
    .def_readonly("joint_limit_max", &RigidBodyTree<double>::joint_limit_max);
>>>>>>> intial

  py::class_<KinematicsCache<double> >(m, "KinematicsCacheDouble");
  py::class_<KinematicsCache<AutoDiffXd> >(m, "KinematicsCacheAutoDiffXd");

<<<<<<< HEAD
=======
  py::class_<RigidBody<double> >(m, "RigidBody")
    .def("get_name", &RigidBody<double>::get_name)
    .def("get_body_index", &RigidBody<double>::get_body_index)
    .def("get_center_of_mass", &RigidBody<double>::get_center_of_mass)
    .def("get_visual_elements", &RigidBody<double>::get_visual_elements)
    .def("AddVisualElement", &RigidBody<double>::AddVisualElement);

>>>>>>> intial
  py::class_<RigidBodyFrame<double>,
             shared_ptr<RigidBodyFrame<double> > >(m, "RigidBodyFrame")
    .def(
        py::init<
            const std::string&,
            RigidBody<double>*,
            const Eigen::VectorXd&,
            const Eigen::VectorXd&>(),
        py::arg("name"), py::arg("body"),
        py::arg("xyz") = Eigen::Vector3d::Zero(),
        py::arg("rpy") = Eigen::Vector3d::Zero())
    .def(
        py::init<
            const std::string&,
            RigidBody<double>*,
            const Eigen::Isometry3d&>(),
        py::arg("name"), py::arg("body"),
        py::arg("transform_to_body"))
    .def("get_name", &RigidBodyFrame<double>::get_name)
    .def("get_frame_index", &RigidBodyFrame<double>::get_frame_index)
    .def("get_rigid_body", &RigidBodyFrame<double>::get_rigid_body,
         py_reference,
         // Keep alive: `this` keeps `return` alive.
         py::keep_alive<1, 0>())
    .def("get_transform_to_body",
         &RigidBodyFrame<double>::get_transform_to_body);

  m.def("AddModelInstanceFromUrdfFile",
<<<<<<< HEAD
        [](const std::string& urdf_filename,
           const FloatingBaseType floating_base_type,
           shared_ptr<RigidBodyFrame<double>> weld_to_frame,
           RigidBodyTree<double>* tree,
           bool do_compile) {
          return parsers::urdf::AddModelInstanceFromUrdfFile(
              urdf_filename, floating_base_type, weld_to_frame,
              do_compile, tree);
        },
        py::arg("urdf_filename"), py::arg("floating_base_type"),
        py::arg("weld_to_frame"), py::arg("tree"),
        py::arg("do_compile") = true);
=======
        py::overload_cast<const std::string&, const FloatingBaseType,
                          shared_ptr<RigidBodyFrame<double>>,
                          RigidBodyTree<double>*>(
            &parsers::urdf::AddModelInstanceFromUrdfFile),
        py::arg("urdf_filename"), py::arg("floating_base_type"),
        py::arg("weld_to_frame"), py::arg("tree"));
>>>>>>> intial
  m.def("AddModelInstanceFromUrdfStringSearchingInRosPackages",
        py::overload_cast<const std::string&, const PackageMap&,
                          const std::string&, const FloatingBaseType,
                          shared_ptr<RigidBodyFrame<double>>,
                          RigidBodyTree<double>*>(
            &parsers::urdf::
                AddModelInstanceFromUrdfStringSearchingInRosPackages));
<<<<<<< HEAD
  m.def("AddModelInstancesFromSdfFile",
        [](const std::string& sdf_filename,
           const FloatingBaseType floating_base_type,
           std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
           RigidBodyTree<double>* tree, bool do_compile) {
          return sdf::AddModelInstancesFromSdfFile(
              sdf_filename, floating_base_type, weld_to_frame, do_compile,
              tree);
        },
        py::arg("sdf_filename"), py::arg("floating_base_type"),
        py::arg("weld_to_frame"), py::arg("tree"),
        py::arg("do_compile") = true);
=======
>>>>>>> intial
  m.def("AddModelInstancesFromSdfString",
        py::overload_cast<const std::string&, const FloatingBaseType,
                          shared_ptr<RigidBodyFrame<double>>,
                          RigidBodyTree<double>*>(
            &sdf::AddModelInstancesFromSdfString));
  m.def("AddModelInstancesFromSdfStringSearchingInRosPackages",
        py::overload_cast<
            const std::string&, const PackageMap&, const FloatingBaseType,
            shared_ptr<RigidBodyFrame<double>>, RigidBodyTree<double>*>(
            &sdf::AddModelInstancesFromSdfStringSearchingInRosPackages)),
  m.def("AddFlatTerrainToWorld", &multibody::AddFlatTerrainToWorld,
        py::arg("tree"), py::arg("box_size") = 1000,
        py::arg("box_depth") = 10);
<<<<<<< HEAD

  py::class_<RigidBodyActuator>(m, "RigidBodyActuator")
    .def_readonly("name", &RigidBodyActuator::name_)
    .def_readonly("body", &RigidBodyActuator::body_)
    .def_readonly("reduction", &RigidBodyActuator::reduction_)
    .def_readonly("effort_limit_min", &RigidBodyActuator::effort_limit_min_)
    .def_readonly("effort_limit_max", &RigidBodyActuator::effort_limit_max_);
=======
>>>>>>> intial
}

}  // namespace pydrake
}  // namespace drake
