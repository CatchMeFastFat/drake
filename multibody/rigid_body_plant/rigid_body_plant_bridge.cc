#include "drake/multibody/rigid_body_plant/rigid_body_plant_bridge.h"

#include <memory>
<<<<<<< HEAD
#include <string>
=======
>>>>>>> intial
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/visual_material.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

<<<<<<< HEAD
using geometry::Box;
using geometry::Cylinder;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::Mesh;
using geometry::SceneGraph;
=======
using geometry::Cylinder;
using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::Mesh;
>>>>>>> intial
using geometry::Shape;
using geometry::Sphere;
using geometry::VisualMaterial;

template <typename T>
<<<<<<< HEAD
RigidBodyPlantBridge<T>::RigidBodyPlantBridge(const RigidBodyTree<T>* tree,
                                              SceneGraph<T>* scene_graph)
    : tree_(tree) {
  DRAKE_THROW_UNLESS(tree_ != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource(this->get_name());
=======
RigidBodyPlantBridge<T>::RigidBodyPlantBridge(
    const RigidBodyTree<T>* tree, GeometrySystem<T>* geometry_system)
    : tree_(tree) {
  DRAKE_THROW_UNLESS(tree_ != nullptr);
  DRAKE_THROW_UNLESS(geometry_system != nullptr);
  source_id_ = geometry_system->RegisterSource(this->get_name());
>>>>>>> intial

  // Declare the tree's pose input port -- don't need the index, it is always 0.
  const int vector_size =
      tree->get_num_positions() + tree->get_num_velocities();
  plant_state_port_ =
      this->DeclareInputPort(kVectorValued, vector_size).get_index();
<<<<<<< HEAD
  RegisterTree(scene_graph);

  // Now that the frames have been registered, instantiate the output port.
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          FramePoseVector<T>(source_id_, body_ids_),
          &RigidBodyPlantBridge::CalcFramePoseOutput)
      .get_index();
=======

  geometry_id_port_ = this->DeclareAbstractOutputPort(
                              &RigidBodyPlantBridge::AllocateFrameIdOutput,
                              &RigidBodyPlantBridge::CalcFrameIdOutput)
                          .get_index();
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
                                &RigidBodyPlantBridge::AllocateFramePoseOutput,
                                &RigidBodyPlantBridge::CalcFramePoseOutput)
                            .get_index();
  RegisterTree(geometry_system);
}

template <typename T>
const OutputPort<T>& RigidBodyPlantBridge<T>::geometry_id_output_port() const {
  return this->get_output_port(geometry_id_port_);
>>>>>>> intial
}

template <typename T>
const OutputPort<T>& RigidBodyPlantBridge<T>::geometry_pose_output_port()
    const {
  return this->get_output_port(geometry_pose_port_);
}

template <typename T>
<<<<<<< HEAD
const InputPort<T>&
=======
const InputPortDescriptor<T>&
>>>>>>> intial
RigidBodyPlantBridge<T>::rigid_body_plant_state_input_port() const {
  return this->get_input_port(plant_state_port_);
}

template <typename T>
<<<<<<< HEAD
void RigidBodyPlantBridge<T>::RegisterTree(SceneGraph<T>* scene_graph) {
=======
void RigidBodyPlantBridge<T>::RegisterTree(GeometrySystem<T>* geometry_system) {
>>>>>>> intial
  // TODO(SeanCurtis-TRI): This treats all bodies in the tree as dynamic. Some
  // may be fixed to the world. In that case, the bodies should *not* be
  // registered, and the geometries should be registered as anchored.
  // This *may* lead to a correctness issues. If two bodies are both rigidly
  // fixed, then their geometries should *implicitly* not be considered for
  // collision. However, if both are included as dynamic bodies, penetrations
  // will be reported.

<<<<<<< HEAD
  using std::make_unique;

=======
>>>>>>> intial
  // Load *dynamic* geometry
  const int body_count = static_cast<int>(tree_->get_bodies().size());
  if (body_count > 1) {  // more than just the world.
    body_ids_.reserve(body_count - 1);
    for (int i = 1; i < body_count; ++i) {
      const RigidBody<T>& body = *tree_->get_bodies()[i];
<<<<<<< HEAD
      // TODO(SeanCurtis-TRI): Possibly account for the fact that some frames
      // may be rigidly affixed to other frames or frames without geometry
      // likewise wouldn't be registered.
      FrameId body_id = scene_graph->RegisterFrame(
=======
      FrameId body_id = geometry_system->RegisterFrame(
>>>>>>> intial
          source_id_,
          GeometryFrame(body.get_name(), Isometry3<double>::Identity(),
                        body.get_model_instance_id()));
      body_ids_.push_back(body_id);
      // TODO(SeanCurtis-TRI): Handle collision and visual elements differently.
      // For now, we're simply consuming the visual elements.
<<<<<<< HEAD
      int visual_count = 0;
=======
>>>>>>> intial
      for (const auto& visual_element : body.get_visual_elements()) {
        std::unique_ptr<Shape> shape;
        Isometry3<double> X_FG = visual_element.getLocalTransform();
        const DrakeShapes::Geometry& geometry = visual_element.getGeometry();
        switch (visual_element.getShape()) {
<<<<<<< HEAD
          case DrakeShapes::BOX: {
            const auto& box = dynamic_cast<const DrakeShapes::Box&>(geometry);
            shape = make_unique<Box>(box.size(0), box.size(1), box.size(2));
            break;
          }
          case DrakeShapes::SPHERE: {
            const auto& sphere =
                dynamic_cast<const DrakeShapes::Sphere&>(geometry);
            shape = make_unique<Sphere>(sphere.radius);
=======
          case DrakeShapes::SPHERE: {
            const auto& sphere =
                dynamic_cast<const DrakeShapes::Sphere&>(geometry);
            shape = std::make_unique<Sphere>(sphere.radius);
>>>>>>> intial
            break;
          }
          case DrakeShapes::CYLINDER: {
            const auto& cylinder =
                dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
<<<<<<< HEAD
            shape = make_unique<Cylinder>(cylinder.radius, cylinder.length);
=======
            shape =
                std::make_unique<Cylinder>(cylinder.radius, cylinder.length);
>>>>>>> intial
            break;
          }
          case DrakeShapes::MESH: {
            const auto& mesh = dynamic_cast<const DrakeShapes::Mesh&>(geometry);
            if (mesh.uri_.find("package://") == 0) {
<<<<<<< HEAD
              shape = make_unique<Mesh>(mesh.uri_);
            } else {
              shape = make_unique<Mesh>(mesh.resolved_filename_);
=======
              shape = std::make_unique<Mesh>(mesh.uri_);
            } else {
              shape = std::make_unique<Mesh>(mesh.resolved_filename_);
>>>>>>> intial
            }
            break;
          }
          default:
            drake::log()->warn("Only spheres, cylinders, and (limited) meshes"
                               "are supported by RigidBodyPlantBridge");
        }
        if (shape) {
          // Visual element's "material" is simply the diffuse rgba values.
          const Vector4<double>& diffuse = visual_element.getMaterial();
<<<<<<< HEAD
          const std::string name = "visual_" + std::to_string(visual_count++);
          scene_graph->RegisterGeometry(
              source_id_, body_id,
              std::make_unique<GeometryInstance>(X_FG, std::move(shape), name,
                                                 VisualMaterial(diffuse)));
=======
          geometry_system->RegisterGeometry(
              source_id_, body_id,
              std::make_unique<GeometryInstance>(
                  X_FG, std::move(shape),
                  VisualMaterial(diffuse)));
>>>>>>> intial
          DRAKE_DEMAND(shape == nullptr);
        }
      }
    }
  }

  // TODO(SeanCurtis-TRI): Handle geometry attached to the world.
}

template <typename T>
<<<<<<< HEAD
void RigidBodyPlantBridge<T>::CalcFramePoseOutput(
    const MyContext& context, FramePoseVector<T>* poses) const {
  DRAKE_DEMAND(source_id_.is_valid());
  DRAKE_DEMAND(poses->size() == static_cast<int>(body_ids_.size()));
=======
FramePoseVector<T> RigidBodyPlantBridge<T>::AllocateFramePoseOutput() const {
  DRAKE_DEMAND(source_id_.is_valid());
  // Poses of the registered bodies in the world -- defaults to identity.
  std::vector<Isometry3<T>> X_WF(body_ids_.size(), Isometry3<T>::Identity());
  return FramePoseVector<T>(source_id_, X_WF);
}

template <typename T>
void RigidBodyPlantBridge<T>::CalcFramePoseOutput(
    const MyContext& context, FramePoseVector<T>* poses) const {
  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  DRAKE_ASSERT(poses->vector().size() == body_ids_.size());
>>>>>>> intial

  const BasicVector<T>& input_vector = *this->EvalVectorInput(context, 0);
  // Obtains the generalized positions from vector_base.
  const VectorX<T> q = input_vector.CopyToVector().head(
      tree_->get_num_positions());

  // Computes the poses of each body.
  KinematicsCache<T> cache = tree_->doKinematics(q);

  // Saves the poses of each body in the frame pose vector
  // NOTE: Body 0 is the world; we skip it.
  // TODO(SeanCurtis-TRI): When I start skipping rigidly fixed bodies, modify
  // this loop to account for them.
  const int world_body = 0;
<<<<<<< HEAD
  poses->clear();
  // NOTE: This relies on the definition that body i has its frame id at i - 1.
  // When we start skipping welded frames, or frames without geometry, this
  // mapping won't be so trivial.
  for (size_t i = 1; i < tree_->get_bodies().size(); ++i) {
    poses->set_value(body_ids_[i - 1],
                     tree_->relativeTransform(cache, world_body, i));
  }
}

=======
  for (size_t i = 1; i < tree_->get_bodies().size(); ++i) {
    pose_data[i - 1] = tree_->relativeTransform(cache, world_body, i);
  }
}

template <typename T>
FrameIdVector RigidBodyPlantBridge<T>::AllocateFrameIdOutput() const {
  DRAKE_DEMAND(source_id_.is_valid());
  FrameIdVector ids(source_id_, body_ids_);
  return ids;
}

template <typename T>
void RigidBodyPlantBridge<T>::CalcFrameIdOutput(
    const MyContext&, FrameIdVector* frame_ids) const {
  DRAKE_DEMAND(source_id_.is_valid());
  *frame_ids = FrameIdVector(source_id_, body_ids_);
}

>>>>>>> intial
// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlantBridge<double>;

}  // namespace systems
}  // namespace drake
