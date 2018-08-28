#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
<<<<<<< HEAD
#include "drake/systems/framework/fixed_input_port_value.h"
=======
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/input_port_value.h"
>>>>>>> intial
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

<<<<<<< HEAD
/** %LeafContext contains all prerequisite data necessary to uniquely determine
the results of computations performed by the associated LeafSystem.
@see Context for more information.

@tparam T The mathematical type of the context, which must be a valid Eigen
          scalar. */
=======
/// %LeafContext contains all prerequisite data necessary to uniquely determine
/// the results of computations performed by the associated LeafSystem.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
>>>>>>> intial
template <typename T>
class LeafContext : public Context<T> {
 public:
  /// @name  Does not allow copy, move, or assignment.
  //@{
  // Copy constructor is protected for use in implementing Clone().
  LeafContext(LeafContext&&) = delete;
  LeafContext& operator=(const LeafContext&) = delete;
  LeafContext& operator=(LeafContext&&) = delete;
  //@}

  LeafContext()
<<<<<<< HEAD
      : state_(std::make_unique<State<T>>()) {}
  ~LeafContext() override {}

#ifndef DRAKE_DOXYGEN_CXX
  // Temporarily promoting these to public so that LeafSystem and testing code
  // can construct a LeafContext with state & parameters. Users should never
  // call these because state & parameters should not be resized once allocated
  // (or at least should be done under Framework control so that dependency
  // tracking can be correctly revised).
  // TODO(sherm1) Make these inaccessible to users. See discussion in PR #9029.
  using Context<T>::init_continuous_state;
  using Context<T>::init_discrete_state;
  using Context<T>::init_abstract_state;
  using Context<T>::init_parameters;
#endif
=======
      : state_(std::make_unique<State<T>>()),
        parameters_(std::make_unique<Parameters<T>>()) {}
  ~LeafContext() override {}

  /// Removes all the input ports, and deregisters them from the output ports
  /// on which they depend.
  void ClearInputPorts() { input_values_.clear(); }

  /// Clears the input ports and allocates @p n new input ports, not connected
  /// to anything.
  void SetNumInputPorts(int n) {
    ClearInputPorts();
    input_values_.resize(n);
  }

  int get_num_input_ports() const override {
    return static_cast<int>(input_values_.size());
  }

  const State<T>& get_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& get_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_.get();
  }

  // =========================================================================
  // Accessors and Mutators for Parameters.

  /// Sets the parameters to @p params, deleting whatever was there before.
  void set_parameters(std::unique_ptr<Parameters<T>> params) {
    parameters_ = std::move(params);
  }

  /// Returns the entire Parameters object.
  const Parameters<T>& get_parameters() const final {
    return *parameters_;
  }

  /// Returns the entire Parameters object.
  Parameters<T>& get_mutable_parameters() final {
    return *parameters_;
  }
>>>>>>> intial

 protected:
  /// Protected copy constructor takes care of the local data members and
  /// all base class members, but doesn't update base class pointers so is
  /// not a complete copy.
  LeafContext(const LeafContext& source) : Context<T>(source) {
    // Make a deep copy of the state.
    state_ = source.CloneState();

<<<<<<< HEAD
=======
    // Make deep copies of the parameters.
    set_parameters(source.parameters_->Clone());

    // Make deep copies of the inputs into FreestandingInputPortValues.
    // TODO(david-german-tri): Preserve version numbers as well.
    for (const auto& port : source.input_values_) {
      if (port == nullptr) {
        input_values_.emplace_back(nullptr);
      } else {
        input_values_.emplace_back(new FreestandingInputPortValue(
            port->get_abstract_data()->Clone()));
      }
    }

>>>>>>> intial
    // Everything else was handled by the Context<T> copy constructor.
  }

  /// Derived classes should reimplement and replace this; don't recursively
  /// invoke it.
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const override {
    return std::unique_ptr<ContextBase>(new LeafContext<T>(*this));
  }

  std::unique_ptr<State<T>> DoCloneState() const override {
    auto clone = std::make_unique<State<T>>();

    // Make a deep copy of the continuous state using BasicVector::Clone().
    const ContinuousState<T>& xc = this->get_continuous_state();
    const int num_q = xc.get_generalized_position().size();
    const int num_v = xc.get_generalized_velocity().size();
    const int num_z = xc.get_misc_continuous_state().size();
    const BasicVector<T>& xc_vector =
        dynamic_cast<const BasicVector<T>&>(xc.get_vector());
    clone->set_continuous_state(std::make_unique<ContinuousState<T>>(
        xc_vector.Clone(), num_q, num_v, num_z));

    // Make deep copies of the discrete and abstract states.
<<<<<<< HEAD
    clone->set_discrete_state(state_->get_discrete_state().Clone());
    clone->set_abstract_state(state_->get_abstract_state().Clone());
=======
    clone->set_discrete_state(get_state().get_discrete_state().Clone());
    clone->set_abstract_state(get_state().get_abstract_state().Clone());
>>>>>>> intial

    return clone;
  }

<<<<<<< HEAD
 private:
  friend class LeafContextTest;
  using ContextBase::AddInputPort;    // For LeafContextTest.
  using ContextBase::AddOutputPort;
  using ContextBase::AddDiscreteStateTicket;
  using ContextBase::AddAbstractStateTicket;
  using ContextBase::AddNumericParameterTicket;
  using ContextBase::AddAbstractParameterTicket;

  const State<T>& do_access_state() const final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  State<T>& do_access_mutable_state() final {
    DRAKE_ASSERT(state_ != nullptr);
    return *state_;
  }

  // The state values (x) for this LeafContext; this is never null.
  std::unique_ptr<State<T>> state_;
=======
  const InputPortValue* GetInputPortValue(int index) const override {
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    return input_values_[index].get();
  }

 private:
  void SetInputPortValue(int index,
                         std::unique_ptr<InputPortValue> port) final {
    DRAKE_DEMAND(index >= 0 && index < get_num_input_ports());
    input_values_[index] = std::move(port);
  }

  // The external inputs to the System.
  std::vector<std::unique_ptr<InputPortValue>> input_values_;

  // The internal state of the System.
  std::unique_ptr<State<T>> state_;

  // The parameters of the system.
  std::unique_ptr<Parameters<T>> parameters_;
>>>>>>> intial
};

}  // namespace systems
}  // namespace drake
