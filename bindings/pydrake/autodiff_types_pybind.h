#pragma once

#include <Eigen/Core>
<<<<<<< HEAD
#include "pybind11/eigen.h"
=======
>>>>>>> intial
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/autodiff.h"

// The macro `PYBIND11_NUMPY_OBJECT_DTYPE` place symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

PYBIND11_NUMPY_OBJECT_DTYPE(drake::AutoDiffXd);
