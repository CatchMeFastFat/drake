# -*- python -*-

def drake_py_library(
        name,
        deps = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
<<<<<<< HEAD

=======
>>>>>>> intial
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
    native.py_library(
        name = name,
        deps = deps,
<<<<<<< HEAD
        **kwargs
    )

def _disable_test_impl(ctx):
    info = dict(
        bad_target = ctx.attr.bad_target,
        good_target = ctx.attr.good_target,
    )
    content = """#!/bin/bash
echo "ERROR: Please use '{good_target}'; the label '{bad_target}'" \
     "has been removed." >&2
exit 1
""".format(**info)
    ctx.actions.write(
        output = ctx.outputs.executable,
        content = content,
    )
    return [DefaultInfo()]

# Defines a test which will fail when run via `bazel run` or `bazel test`,
# pointing the user to the correct binary to use. This should typically have
# a "manual" tag.
_disable_test = rule(
    attrs = {
        "bad_target": attr.string(mandatory = True),
        "good_target": attr.string(mandatory = True),
    },
    test = True,
    implementation = _disable_test_impl,
)
=======
        **kwargs)
>>>>>>> intial

def _py_target_isolated(
        name,
        py_target = None,
        srcs = None,
        main = None,
        isolate = True,
        visibility = None,
        **kwargs):
    # See #8041 for more details.
    if py_target == None:
        fail("Must supply macro function for defining `py_target`.")
<<<<<<< HEAD

    # Do not isolate targets that are already isolated. This generally happens
    # when linting tests (which are isolated) are invoked for isolated Python
    # targets. Without this check, the actual test turns into
    # `py/py/{name}`.
    prefix = "py/"
    if isolate and not name.startswith(prefix):
        actual = prefix + name

=======
    # Do not isolate targets that are already isolated. This generally happens
    # when linting tests (which are isolated) are invoked for isolated Python
    # targets. Without this check, the actual test turns into
    # `_isolated/_isolated/{name}`.
    prefix = "_isolated/"
    if isolate and not name.startswith(prefix):
        actual = prefix + name
>>>>>>> intial
        # Preserve original functionality.
        if not main:
            main = name + ".py"
        if not srcs:
            srcs = [name + ".py"]
        py_target(
            name = actual,
            srcs = srcs,
            main = main,
<<<<<<< HEAD
            visibility = visibility,
            **kwargs
        )

        # Disable and redirect original name.
        package_prefix = "//" + native.package_name() + ":"

        # N.B. We make the disabled rule a test, even if the original was not.
        # This ensures that developers will see the redirect using both
        # `bazel run` or `bazel test`.
        _disable_test(
            name = name,
            good_target = package_prefix + actual,
            bad_target = package_prefix + name,
            tags = ["manual"],
=======
            visibility = ["//visibility:private"],
            **kwargs)
        native.alias(
            name = name,
            actual = actual,
>>>>>>> intial
            visibility = visibility,
        )
    else:
        py_target(
            name = name,
            srcs = srcs,
            main = main,
            visibility = visibility,
<<<<<<< HEAD
            **kwargs
        )
=======
            **kwargs)
>>>>>>> intial

def drake_py_binary(
        name,
        srcs = None,
<<<<<<< HEAD
        main = None,
        deps = None,
        isolate = False,
        tags = [],
        add_test_rule = 0,
        test_rule_args = [],
=======
        deps = None,
        isolate = False,
>>>>>>> intial
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is False)
        If True, the binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.
    """
<<<<<<< HEAD

    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = deps or []
    if "//:module_py" not in deps:
        deps += ["//:module_py"]
    if main == None and len(srcs) == 1:
        main = srcs[0]
=======
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
>>>>>>> intial
    _py_target_isolated(
        name = name,
        py_target = native.py_binary,
        isolate = isolate,
        srcs = srcs,
<<<<<<< HEAD
        main = main,
        deps = deps,
        tags = tags,
        **kwargs
    )
    if add_test_rule:
        drake_py_test(
            name = name + "_test",
            srcs = srcs,
            main = main,
            deps = deps,
            isolate = isolate,
            args = test_rule_args,
            tags = tags + ["nolint"],
            # N.B. Same as the warning in `drake_pybind_cc_googletest`: numpy
            # imports unittest unconditionally.
            allow_import_unittest = True,
            **kwargs
        )
=======
        deps = deps,
        **kwargs)
>>>>>>> intial

def drake_py_unittest(
        name,
        srcs = [],
        **kwargs):
    """Declares a `unittest`-based python test.

    This macro should be preferred instead of the basic drake_py_test for tests
    that use the `unittest` framework.  Tests that use this macro should *not*
<<<<<<< HEAD
    contain a __main__ handler nor a shebang line.  By default, sets test size
    to "small" to indicate a unit test.
=======
    contain a __main__ handler nor a shebang line.
>>>>>>> intial
    """
    helper = "//common/test_utilities:drake_py_unittest_main.py"
    if not srcs:
        srcs = ["test/%s.py" % name]
    drake_py_test(
        name = name,
        srcs = srcs + [helper],
        main = helper,
        allow_import_unittest = True,
<<<<<<< HEAD
        **kwargs
    )

def drake_py_test(
        name,
        size = None,
=======
        **kwargs)

def drake_py_test(
        name,
>>>>>>> intial
        srcs = None,
        deps = None,
        isolate = True,
        allow_import_unittest = False,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param isolate (optional, default is True)
        If True, the test binary will be placed in a folder isolated from the
        library code. This prevents submodules from leaking in as top-level
        submodules. For more detail, see #8041.

    @param allow_import_unittest (optional, default is False)
        If False, this test (and anything it imports) is prevented from doing
        `import unittest`.  This is a guard against writing `unittest`-based
        cases that accidentally never get run.  In general, `unittest`-based
        tests should use the `drake_py_unittest` macro instead of this one
        (thus disabling this interlock), but can override this parameter in
        case something unique is happening and the other macro can't be used.
<<<<<<< HEAD

    By default, sets test size to "small" to indicate a unit test.
    """
    if size == None:
        size = "small"
    if srcs == None:
        srcs = ["test/%s.py" % name]

    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = deps or []
    if "//:module_py" not in deps:
        deps += ["//:module_py"]
=======
    """
    if srcs == None:
        srcs = ["test/%s.py" % name]
    # Work around https://github.com/bazelbuild/bazel/issues/1567.
    deps = (deps or []) + ["//:module_py"]
>>>>>>> intial
    if not allow_import_unittest:
        deps = deps + ["//common/test_utilities:disable_python_unittest"]
    _py_target_isolated(
        name = name,
        py_target = native.py_test,
        isolate = isolate,
<<<<<<< HEAD
        size = size,
        srcs = srcs,
        deps = deps,
        **kwargs
    )
=======
        srcs = srcs,
        deps = deps,
        **kwargs)
>>>>>>> intial

def py_test_isolated(
        name,
        **kwargs):
    """Provides a directory-isolated Python test, robust against shadowing
    (#8041).
    """
    _py_target_isolated(
        name = name,
        py_target = native.py_test,
        isolate = True,
<<<<<<< HEAD
        **kwargs
    )
=======
        **kwargs)
>>>>>>> intial
