#include "drake/multibody/multibody_tree/rigid_body.h"

#include <memory>

#include "drake/common/autodiff.h"
<<<<<<< HEAD
#include "drake/multibody/multibody_tree/model_instance.h"
=======
>>>>>>> intial

namespace drake {
namespace multibody {

template <typename T>
<<<<<<< HEAD
RigidBody<T>::RigidBody(const SpatialInertia<double>& M)
    : Body<T>("", default_model_instance(), M.get_mass()),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double>& M)
    : Body<T>(body_name, default_model_instance(), M.get_mass()),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        ModelInstanceIndex model_instance,
                        const SpatialInertia<double>& M)
    : Body<T>(body_name, model_instance, M.get_mass()),
=======
RigidBody<T>::RigidBody(const SpatialInertia<double> M) :
    default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double> M)
    : Body<T>(body_name, M.get_mass()),
>>>>>>> intial
      default_spatial_inertia_(M) {}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;
template class RigidBody<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
