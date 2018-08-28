# -*- mode: python -*-

# These macros are intended to be used when declaring tests that either may-use
<<<<<<< HEAD
# or must-use either Gurobi or MOSEK commercial solvers. These labels both
# account for any license-related needs and provide a marker so that
# //tools/bazel.rc can selectively enable tests based on the developer's chosen
# configuration.
=======
# or must-use either Gurobi or Mosek commercial solvers.  These labels both
# account for any license-server specific needs (such as network access or rate
# control), as well as provide a marker so that //tools/bazel.rc can
# selectively enable tests based on the developer's chosen configuration.
>>>>>>> intial

def gurobi_test_tags(gurobi_required = True):
    """Returns the test tags necessary for properly running Gurobi tests.

    By default, sets gurobi_required=True, which will require that the supplied
    tag filters include "gurobi".

<<<<<<< HEAD
    Gurobi checks a license file outside the workspace so tests that use Gurobi
    must have the tag "no-sandbox". For the moment, we also require the tag
    "exclusive" to rate-limit license servers with a small number of licenses.
    """

    # TODO(david-german-tri): Find a better fix for the license server problem.
    nominal_tags = [
        "exclusive",  # implies "local"
        "no-sandbox",
=======
    Gurobi checks a license file, and may need to contact a license server to
    check out a license. Therefore, tests that use Gurobi must have the tag
    "local", because they are non-hermetic. For the moment, we also require
    the tag "exclusive", to rate-limit license servers with a small number of
    licenses.
    """
    # TODO(david-german-tri): Find a better fix for the license server problem.
    nominal_tags = [
        "exclusive",
        "local",
>>>>>>> intial
    ]
    if gurobi_required:
        return nominal_tags + ["gurobi"]
    else:
        return nominal_tags

def mosek_test_tags(mosek_required = True):
<<<<<<< HEAD
    """Returns the test tags necessary for properly running MOSEK tests.
=======
    """Returns the test tags necessary for properly running mosek tests.
>>>>>>> intial

    By default, sets mosek_required=True, which will require that the supplied
    tag filters include "mosek".

<<<<<<< HEAD
    MOSEK checks a license file outside the workspace, so tests that use MOSEK
    must have the tag "no-sandbox".
    """
    nominal_tags = [
        "no-sandbox",
=======
    MOSEK checks a license file, and may need to contact a license server to
    check out a license. Therefore, tests that use MOSEK must have the tag
    "local", because they are non-hermetic.
    """
    nominal_tags = [
        "local",
>>>>>>> intial
    ]
    if mosek_required:
        return nominal_tags + ["mosek"]
    else:
        return nominal_tags
