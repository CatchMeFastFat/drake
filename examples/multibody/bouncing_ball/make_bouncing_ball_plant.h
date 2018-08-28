#pragma once

#include <memory>

<<<<<<< HEAD
#include "drake/geometry/scene_graph.h"
=======
#include "drake/geometry/geometry_system.h"
>>>>>>> intial
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

/// This method makes a MultibodyPlant model of a ball falling into a plane.
/// MultibodyPlant models the contact of the ball with the ground as a perfectly
/// inelastic collision (zero coefficient of restitution), i.e. energy is lost
/// due to the collision.
///
/// @param[in] radius
///   The radius of the ball.
/// @param[in] mass
///   The mass of the ball.
<<<<<<< HEAD
/// @param[in] surface_friction
///   The Coulomb's law coefficients of friction.
/// @param[in] gravity_W
///   The acceleration of gravity vector, expressed in the world frame W.
/// @param scene_graph
///   If a SceneGraph is provided with this argument, this factory method
=======
/// @param[in] gravity_W
///   The acceleration of gravity vector, expressed in the world frame W.
/// @param geometry_system
///   If a GeometrySystem is provided with this argument, this factory method
>>>>>>> intial
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for collision.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeBouncingBallPlant(
<<<<<<< HEAD
    double radius, double mass,
    const drake::multibody::multibody_plant::CoulombFriction<double>&
        surface_friction,
    const Vector3<double>& gravity_W,
    geometry::SceneGraph<double>* scene_graph = nullptr);
=======
    double radius, double mass, const Vector3<double>& gravity_W,
    geometry::GeometrySystem<double>* geometry_system = nullptr);
>>>>>>> intial

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
