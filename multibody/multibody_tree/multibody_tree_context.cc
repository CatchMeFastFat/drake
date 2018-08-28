#include "drake/multibody/multibody_tree/multibody_tree_context.h"

<<<<<<< HEAD
#include "drake/common/default_scalars.h"

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyTreeContext)
=======
#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {

// Explicitly instantiates on the most common scalar types.
template class MultibodyTreeContext<double>;
template class MultibodyTreeContext<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
>>>>>>> intial
