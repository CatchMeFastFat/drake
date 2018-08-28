#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
<<<<<<< HEAD
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
=======
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
>>>>>>> intial
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {

<<<<<<< HEAD
using geometry::SceneGraph;
=======
using geometry::GeometrySystem;
>>>>>>> intial
using lcm::DrakeLcm;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::multibody_plant::MultibodyPlant;
<<<<<<< HEAD
using multibody::parsing::AddModelFromSdfFile;
using multibody::JointActuator;
using multibody::RevoluteJoint;
using multibody::UniformGravityFieldElement;
using systems::Context;
=======
using multibody::RevoluteJoint;
using systems::Context;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;
>>>>>>> intial

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

<<<<<<< HEAD
DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");

// This helper method makes an LQR controller to balance an acrobot model
// specified in the SDF file `file_name`.
std::unique_ptr<systems::AffineSystem<double>> MakeBalancingLQRController(
    const std::string &file_name) {
  const std::string full_name = FindResourceOrThrow(file_name);
  // LinearQuadraticRegulator() below requires the controller's model of the
  // plant to only have a single input port corresponding to the actuation.
  // Therefore we create a new model that meets this requirement. (a model
  // created along with a SceneGraph for simulation would also have input ports
  // to interact with that SceneGraph).
  MultibodyPlant<double> acrobot;
  AddModelFromSdfFile(full_name, &acrobot);
  // Add gravity to the model.
  acrobot.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());
  // We are done defining the model.
  acrobot.Finalize();

  const RevoluteJoint<double>& shoulder =
      acrobot.GetJointByName<RevoluteJoint>("ShoulderJoint");
  const RevoluteJoint<double>& elbow =
      acrobot.GetJointByName<RevoluteJoint>("ElbowJoint");
  std::unique_ptr<Context<double>> context = acrobot.CreateDefaultContext();
=======
std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotParameters& acrobot_parameters) {
  std::unique_ptr<const MultibodyPlant<double>> acrobot =
      MakeAcrobotPlant(acrobot_parameters, true);
  const RevoluteJoint<double>& shoulder =
      acrobot->GetJointByName<RevoluteJoint>(
          acrobot_parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      acrobot->GetJointByName<RevoluteJoint>(
          acrobot_parameters.elbow_joint_name());
  std::unique_ptr<Context<double>> context = acrobot->CreateDefaultContext();
>>>>>>> intial

  // Set nominal actuation torque to zero.
  context->FixInputPort(0, Vector1d::Constant(0.0));

  shoulder.set_angle(context.get(), M_PI);
  shoulder.set_angular_rate(context.get(), 0.0);
  elbow.set_angle(context.get(), 0.0);
  elbow.set_angular_rate(context.get(), 0.0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  return systems::controllers::LinearQuadraticRegulator(
<<<<<<< HEAD
      acrobot, *context, Q, R);
=======
      *acrobot, *context, Q, R);
>>>>>>> intial
}

int do_main() {
  systems::DiagramBuilder<double> builder;

<<<<<<< HEAD
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double simulation_time = FLAGS_simulation_time;

  const double time_step = FLAGS_time_stepping ? 1.0e-3 : 0.0;

  // Make and add the acrobot model.
  const std::string relative_name =
      "drake/multibody/benchmarks/acrobot/acrobot.sdf";
  const std::string full_name = FindResourceOrThrow(relative_name);
  MultibodyPlant<double>& acrobot =
      *builder.AddSystem<MultibodyPlant>(time_step);

  AddModelFromSdfFile(full_name, &acrobot, &scene_graph);

  // Add gravity to the model.
  acrobot.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // We are done defining the model.
  acrobot.Finalize(&scene_graph);

  DRAKE_DEMAND(acrobot.num_actuators() == 1);
  DRAKE_DEMAND(acrobot.num_actuated_dofs() == 1);

  const RevoluteJoint<double>& shoulder =
      acrobot.GetJointByName<RevoluteJoint>("ShoulderJoint");
  const RevoluteJoint<double>& elbow =
      acrobot.GetJointByName<RevoluteJoint>("ElbowJoint");

  // Drake's parser will default the name of the actuator to match the name of
  // the joint it actuates.
  const JointActuator<double>& actuator =
      acrobot.GetJointActuatorByName("ElbowJoint");
  DRAKE_DEMAND(actuator.joint().name() == "ElbowJoint");
=======
  GeometrySystem<double>& geometry_system =
      *builder.AddSystem<GeometrySystem>();
  geometry_system.set_name("geometry_system");

  const double simulation_time = FLAGS_simulation_time;

  // Make and add the acrobot model.
  const AcrobotParameters acrobot_parameters;
  const MultibodyPlant<double>& acrobot =
      *builder.AddSystem(
          MakeAcrobotPlant(
              acrobot_parameters, true /* Finalize the plant */,
              &geometry_system));
  const RevoluteJoint<double>& shoulder =
      acrobot.GetJointByName<RevoluteJoint>(
          acrobot_parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      acrobot.GetJointByName<RevoluteJoint>(
          acrobot_parameters.elbow_joint_name());
>>>>>>> intial

  // For this example the controller's model of the plant exactly matches the
  // plant to be controlled (in reality there would always be a mismatch).
  auto controller = builder.AddSystem(
<<<<<<< HEAD
      MakeBalancingLQRController(relative_name));
=======
      BalancingLQRController(acrobot_parameters));
>>>>>>> intial
  controller->set_name("controller");
  builder.Connect(acrobot.get_continuous_state_output_port(),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot.get_actuation_input_port());

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
  DRAKE_DEMAND(!!acrobot.get_source_id());

  builder.Connect(
<<<<<<< HEAD
      acrobot.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(acrobot.get_source_id().value()));

  // Last thing before building the diagram; configure the system for
  // visualization.
  DrakeLcm lcm;
  geometry::ConnectVisualization(scene_graph, &builder, &lcm);
  auto diagram = builder.Build();

  // Load message must be sent before creating a Context.
  geometry::DispatchLoadMessage(scene_graph, &lcm);
=======
      acrobot.get_geometry_ids_output_port(),
      geometry_system.get_source_frame_id_port(
          acrobot.get_source_id().value()));
  builder.Connect(
      acrobot.get_geometry_poses_output_port(),
      geometry_system.get_source_pose_port(acrobot.get_source_id().value()));

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
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(acrobot, diagram_context.get());

  // Set an initial condition near the upright fixed point.
  shoulder.set_angle(&acrobot_context, M_PI + 0.1);
  shoulder.set_angular_rate(&acrobot_context, 0.0);
  elbow.set_angle(&acrobot_context, -0.1);
  elbow.set_angular_rate(&acrobot_context, 0.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyPlant with "
      "LQR stabilization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::acrobot::do_main();
}
