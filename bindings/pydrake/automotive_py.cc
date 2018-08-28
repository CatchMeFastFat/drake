#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/calc_ongoing_road_position.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
<<<<<<< HEAD
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/road_odometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"
=======
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/simple_car.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
>>>>>>> intial
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(automotive, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::automotive;

  m.doc() = "Bindings for Automotive systems";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.rendering");

  using T = double;

<<<<<<< HEAD
  py::enum_<AheadOrBehind>(m, "AheadOrBehind")
      .value("kAhead", AheadOrBehind::kAhead)
      .value("kBehind", AheadOrBehind::kBehind);

=======
>>>>>>> intial
  py::enum_<RoadPositionStrategy>(m, "RoadPositionStrategy")
      .value("kCache", RoadPositionStrategy::kCache)
      .value("kExhaustiveSearch", RoadPositionStrategy::kExhaustiveSearch);

  py::enum_<ScanStrategy>(m, "ScanStrategy")
      .value("kPath", ScanStrategy::kPath)
      .value("kBranches", ScanStrategy::kBranches);

<<<<<<< HEAD
  py::class_<ClosestPose<T>>(m, "ClosestPose")
      .def(py::init<>())
      .def(py::init<const RoadOdometry<T>&, const T&>(), py::arg("odom"),
           py::arg("dist"),
           // Keep alive, transitive: `self` keeps `RoadOdometry` pointer
           // members alive.
           py::keep_alive<1, 2>())
      .def_readwrite("odometry", &ClosestPose<T>::odometry,
                     py_reference_internal)
      .def_readwrite("distance", &ClosestPose<T>::distance);

  py::class_<RoadOdometry<T>> road_odometry(m, "RoadOdometry");
  road_odometry
      .def(py::init<>())
      .def(py::init<const maliput::api::RoadPosition&,
                    const systems::rendering::FrameVelocity<T>&>(),
           py::arg("road_position"), py::arg("frame_velocity"),
           // Keep alive, transitive: `self` keeps `RoadPosition` pointer
           // members alive.
           py::keep_alive<1, 2>())
      .def(py::init<const maliput::api::Lane*,
                    const maliput::api::LanePositionT<T>&,
                    const systems::rendering::FrameVelocity<T>&>(),
           py::arg("lane"), py::arg("lane_position"), py::arg("frame_velocity"),
           // Keep alive, reference: `self` keeps `Lane*` alive.
           py::keep_alive<1, 2>())
      .def_readwrite("pos", &RoadOdometry<T>::pos)
      .def_readwrite("vel", &RoadOdometry<T>::vel);
  DefReadWriteKeepAlive(&road_odometry, "lane", &RoadOdometry<T>::lane);

  py::class_<LaneDirection>(m, "LaneDirection")
      .def(py::init<const maliput::api::Lane*, bool>(), py::arg("lane"),
           py::arg("with_s"))
      .def_readwrite("lane", &LaneDirection::lane, py_reference_internal)
=======
  py::class_<LaneDirection>(m, "LaneDirection")
      .def(py::init<const maliput::api::Lane*, bool>(), py::arg("lane"),
           py::arg("with_s"))
      .def_readwrite("lane", &LaneDirection::lane)
>>>>>>> intial
      .def_readwrite("with_s", &LaneDirection::with_s);
  pysystems::AddValueInstantiation<LaneDirection>(m);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<DrivingCommand<T>, BasicVector<T>>(m, "DrivingCommand")
      .def(py::init<>())
      .def("steering_angle", &DrivingCommand<T>::steering_angle)
      .def("acceleration", &DrivingCommand<T>::acceleration)
      .def("set_steering_angle", &DrivingCommand<T>::set_steering_angle)
      .def("set_acceleration", &DrivingCommand<T>::set_acceleration);

  py::class_<IdmController<T>, LeafSystem<T>>(m, "IdmController")
<<<<<<< HEAD
      .def(py::init<const maliput::api::RoadGeometry&, ScanStrategy,
                    RoadPositionStrategy, double>(),
           py::arg("road"), py::arg("path_or_branches"),
           py::arg("road_position_strategy"), py::arg("period_sec"))
=======
      .def(py::init<const maliput::api::RoadGeometry&,
           ScanStrategy, RoadPositionStrategy, double>(), py::arg("road"),
           py::arg("path_or_branches"), py::arg("road_position_strategy"),
           py::arg("period_sec"))
>>>>>>> intial
      .def("ego_pose_input", &IdmController<T>::ego_pose_input,
           py_reference_internal)
      .def("ego_velocity_input", &IdmController<T>::ego_velocity_input,
           py_reference_internal)
      .def("traffic_input", &IdmController<T>::traffic_input,
           py_reference_internal)
      .def("acceleration_output", &IdmController<T>::acceleration_output,
           py_reference_internal);

<<<<<<< HEAD
  py::class_<PoseSelector<T>>(m, "PoseSelector")
      .def_static(
          "FindClosestPair",
          [](const maliput::api::Lane* lane,
             const systems::rendering::PoseVector<T>& ego_pose,
             const systems::rendering::PoseBundle<T>& traffic_poses,
             const T& scan_distance, ScanStrategy path_or_branches) {
            return PoseSelector<T>::FindClosestPair(
                lane, ego_pose, traffic_poses, scan_distance, path_or_branches);
          },
          py::arg("lane"), py::arg("ego_pose"), py::arg("traffic_poses"),
          py::arg("scan_distance"), py::arg("path_or_branches"))
      .def_static("FindSingleClosestPose",
                  [](const maliput::api::Lane* lane,
                     const systems::rendering::PoseVector<T>& ego_pose,
                     const systems::rendering::PoseBundle<T>& traffic_poses,
                     const T& scan_distance, const AheadOrBehind side,
                     ScanStrategy path_or_branches) {
                    return PoseSelector<T>::FindSingleClosestPose(
                        lane, ego_pose, traffic_poses, scan_distance, side,
                        path_or_branches);
                  },
                  py::arg("lane"), py::arg("ego_pose"),
                  py::arg("traffic_poses"), py::arg("scan_distance"),
                  py::arg("side"), py::arg("path_or_branches"))
      .def_static("GetSigmaVelocity", &PoseSelector<T>::GetSigmaVelocity);

  py::class_<PurePursuitController<T>, LeafSystem<T>>(m,
                                                      "PurePursuitController")
=======
  py::class_<PurePursuitController<T>, LeafSystem<T>>(
      m, "PurePursuitController")
>>>>>>> intial
      .def(py::init<>())
      .def("ego_pose_input", &PurePursuitController<T>::ego_pose_input,
           py_reference_internal)
      .def("lane_input", &PurePursuitController<T>::lane_input,
           py_reference_internal)
      .def("steering_command_output",
           &PurePursuitController<T>::steering_command_output,
           py_reference_internal);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
<<<<<<< HEAD
  py::class_<SimpleCarState<T>, BasicVector<T>>(m, "SimpleCarState")
      .def(py::init<>())
      .def("x", &SimpleCarState<T>::x)
      .def("y", &SimpleCarState<T>::y)
      .def("heading", &SimpleCarState<T>::heading)
      .def("velocity", &SimpleCarState<T>::velocity)
      .def("set_x", &SimpleCarState<T>::set_x)
      .def("set_y", &SimpleCarState<T>::set_y)
      .def("set_heading", &SimpleCarState<T>::set_heading)
      .def("set_velocity", &SimpleCarState<T>::set_velocity);
=======
  py::class_<SimpleCarState<T>, BasicVector<T>>(m, "SimpleCarState");
>>>>>>> intial

  py::class_<SimpleCar<T>, LeafSystem<T>>(m, "SimpleCar")
      .def(py::init<>())
      .def("state_output", &SimpleCar<T>::state_output, py_reference_internal)
      .def("pose_output", &SimpleCar<T>::pose_output, py_reference_internal)
      .def("velocity_output", &SimpleCar<T>::velocity_output,
           py_reference_internal);

  // TODO(jadecastro) Bind more systems as appropriate.
}

}  // namespace pydrake
}  // namespace drake
