#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/analysis/initial_value_problem-inl.h"

<<<<<<< HEAD
#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class InitialValueProblem);
=======
namespace drake {
namespace systems {

template class InitialValueProblem<double>;
>>>>>>> intial

}  // namespace systems
}  // namespace drake
