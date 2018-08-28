#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/multibody/bouncing_ball/make_bouncing_ball_plant.h"
<<<<<<< HEAD
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
=======
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
>>>>>>> intial
#include "drake/math/random_rotation.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
<<<<<<< HEAD
=======
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
>>>>>>> intial

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "runge_kutta2",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
<<<<<<< HEAD
using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::ImplicitEulerIntegrator;
using systems::RungeKutta2Integrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::MultibodyTree;
using drake::multibody::QuaternionFloatingMobilizer;
=======
using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::MultibodyTree;
using drake::multibody::QuaternionFloatingMobilizer;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;
using drake::systems::RungeKutta2Integrator;
using drake::systems::RungeKutta3Integrator;
using drake::systems::SemiExplicitEulerIntegrator;
>>>>>>> intial

int do_main() {
  systems::DiagramBuilder<double> builder;

<<<<<<< HEAD
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
=======
  GeometrySystem<double>& geometry_system =
      *builder.AddSystem<GeometrySystem>();
  geometry_system.set_name("geometry_system");
>>>>>>> intial

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 0.001;

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const double z0 = 0.3;        // Initial height.
<<<<<<< HEAD
  const CoulombFriction<double> coulomb_friction(
      0.8 /* static friction */, 0.3 /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeBouncingBallPlant(
      radius, mass, coulomb_friction, -g * Vector3d::UnitZ(), &scene_graph));
=======

  MultibodyPlant<double>& plant =
      *builder.AddSystem(MakeBouncingBallPlant(
          radius, mass, -g * Vector3d::UnitZ(), &geometry_system));
>>>>>>> intial
  const MultibodyTree<double>& model = plant.model();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(0.001);

  // Hint the integrator's time step based on the contact time scale.
  // A fraction of this time scale is used which is chosen so that the fixed
  // time step integrators are stable.
  const double max_time_step =
      plant.get_contact_penalty_method_time_scale() / 30;

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

<<<<<<< HEAD
=======
  // Boilerplate used to connect the plant to a GeometrySystem for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

>>>>>>> intial
  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(
<<<<<<< HEAD
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  // Last thing before building the diagram; configure the system for
  // visualization.
  DrakeLcm lcm;
  geometry::ConnectVisualization(scene_graph, &builder, &lcm);
  auto diagram = builder.Build();

  // Load message must be sent before creating a Context.
  geometry::DispatchLoadMessage(scene_graph, &lcm);
=======
      plant.get_geometry_ids_output_port(),
      geometry_system.get_source_frame_id_port(
          plant.get_source_id().value()));
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      geometry_system.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(geometry_system.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(geometry_system.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(geometry_system);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
>>>>>>> intial

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set at height z0 with random orientation.
  std::mt19937 generator(41);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  model.SetDefaultContext(&plant_context);
  Matrix3d R_WB = math::UniformlyRandomRotationMatrix(&generator).matrix();
  Isometry3d X_WB = Isometry3d::Identity();
  X_WB.linear() = R_WB;
  X_WB.translation() = Vector3d(0.0, 0.0, z0);
  model.SetFreeBodyPoseOrThrow(
      model.GetBodyByName("Ball"), X_WB, &plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>* integrator{nullptr};
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<RungeKutta2Integrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
        "' not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);

  // Error control is only supported for variable time step integrators.
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  // Some sanity checks:
  if (FLAGS_integration_scheme == "semi_explicit_euler") {
    DRAKE_DEMAND(integrator->get_fixed_step_mode() == true);
  }

  // Checks for variable time step integrators.
  if (!integrator->get_fixed_step_mode()) {
    // From IntegratorBase::set_maximum_step_size():
    // "The integrator may stretch the maximum step size by as much as 1% to
    // reach discrete event." Thus the 1.01 factor in this DRAKE_DEMAND.
    DRAKE_DEMAND(
        integrator->get_largest_step_size_taken() <= 1.01 * max_time_step);
    DRAKE_DEMAND(integrator->get_smallest_adapted_step_size_taken() <=
        integrator->get_largest_step_size_taken());
    DRAKE_DEMAND(integrator->get_num_steps_taken() >=
        FLAGS_simulation_time / max_time_step);
  }

  // Checks for fixed time step integrators.
  if (integrator->get_fixed_step_mode()) {
    const int kNumEvaluationsPerStep =
        FLAGS_integration_scheme == "runge_kutta2"? 2 : 1;
    DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
        integrator->get_num_steps_taken() * kNumEvaluationsPerStep);
    DRAKE_DEMAND(
        integrator->get_num_step_shrinkages_from_error_control() == 0);
  }

  // We made a good guess for max_time_step and therefore we expect no
  // failures when taking a time step.
  DRAKE_DEMAND(integrator->get_num_substep_failures() == 0);
  DRAKE_DEMAND(
      integrator->get_num_step_shrinkages_from_substep_failures() == 0);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyTree,"
<<<<<<< HEAD
      "with SceneGraph visualization. "
=======
      "with GeometrySystem visualization. "
>>>>>>> intial
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::bouncing_ball::do_main();
}
