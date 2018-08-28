#!/usr/bin/env python2

"""Command-line tool to generate Drake's Doxygen content.

"""

import argparse
import os
<<<<<<< HEAD
import shutil
=======
>>>>>>> intial
import subprocess
import sys

from collections import OrderedDict
from os.path import dirname

<<<<<<< HEAD
def _get_drake_workspace():
=======
def _get_drake_distro():
>>>>>>> intial
    """Find and return the path to the drake workspace."""

    result = dirname(dirname(os.path.abspath(sys.argv[0])))
    if not os.path.exists(os.path.join(result, "WORKSPACE")):
        raise RuntimeError("Could not place drake at " + result)
    return result

def _run_doxygen(args):
    # Find our programs.
<<<<<<< HEAD
    if sys.platform == "darwin":
        path = "/usr/local/bin:/usr/bin:/bin"
    else:
        path = "/usr/bin:/bin"
    env = {"PATH": path}
    doxygen = subprocess.check_output(["which", "doxygen"], env=env).strip()
    dot = subprocess.check_output(["which", "dot"], env=env).strip()

    # Prepare the input and output folders.  We will copy the requested input
    # file(s) into a temporary scratch directory, so that Doxygen doesn't root
    # around in the drake_workspace (which is extremely slow).
    drake_workspace = _get_drake_workspace()
    binary_dir = os.path.join(drake_workspace, "build/drake/doc")
    input_root = os.path.join(binary_dir, "input")
    if os.path.exists(input_root):
        shutil.rmtree(input_root)
    source_root = os.path.join(input_root, "drake")
    os.makedirs(source_root)
    shutil.copytree(
        os.path.join(drake_workspace, "doc"),
        os.path.join(source_root, "doc"))
    inputs = args.inputs
    if not inputs:
        # No inputs were specified; use everything.
        inputs = [
            os.path.join(drake_workspace, x)
            for x in os.listdir(drake_workspace)
        ]
    for x in inputs:
        # Find the workspace-relative pathname.
        abs_x = os.path.abspath(x)
        rel_x = os.path.relpath(abs_x, drake_workspace)
        assert not rel_x.startswith(".."), rel_x

        # Skip bad things.
        if rel_x.startswith("."): continue
        if rel_x.startswith("bazel"): continue
        if rel_x.startswith("build"): continue
        if rel_x.startswith("doc"): continue  # N.B. Done above.
        if rel_x.startswith("third_party"): continue

        # Copy the workspace files into the input scratch dir.
        target = os.path.join(source_root, rel_x)
        if os.path.isfile(abs_x):
            parent = os.path.dirname(target)
            if not os.path.exists(parent):
                os.makedirs(parent)
            shutil.copy2(abs_x, target)
        else:
            assert os.path.isdir(abs_x)
            # N.B. This won't work if the user redundantly requested both a
            # parent directory and one of its children.  For now, the answer is
            # just "don't do that".
            try:
                shutil.copytree(abs_x, target)
            except OSError, e:
                print(str(e) + " during copytree.  Perhaps you tried to input "
                      "both a parent directory and its child?")
                sys.exit(1)

    # Populate the definitions dict needed by Doxygen_CXX.in.
    definitions = OrderedDict()
    definitions["INPUT_ROOT"] = input_root
=======
    doxygen = subprocess.check_output(["which", "doxygen"]).strip()
    dot = subprocess.check_output(["which", "dot"]).strip()

    # Compute the definitions dict needed by Doxygen_CXX.in.
    drake_distro = _get_drake_distro()
    binary_dir = os.path.join(drake_distro, "build/drake/doc")
    if not os.path.exists(binary_dir):
        os.makedirs(binary_dir)
    definitions = OrderedDict()
    definitions["DRAKE_WORKSPACE_ROOT"] = drake_distro
>>>>>>> intial
    definitions["BINARY_DIR"] = binary_dir
    if args.quick:
        definitions["DOXYGEN_DOT_FOUND"] = "NO"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = ""
    else:
        definitions["DOXYGEN_DOT_FOUND"] = "YES"
        definitions["DOXYGEN_DOT_EXECUTABLE"] = dot
    definition_args = ["-D%s=%s" % (key, value)
                       for key, value in definitions.iteritems()]

    # Create Doxyfile_CXX.
<<<<<<< HEAD
    in_filename = os.path.join(drake_workspace, "doc/Doxyfile_CXX.in")
    doxyfile = os.path.join(binary_dir, "Doxyfile_CXX")
    subprocess.check_call(
        [os.path.join(
            drake_workspace, "tools/workspace/cmake_configure_file.py"),
=======
    in_filename = os.path.join(drake_distro, "doc/Doxyfile_CXX.in")
    doxyfile = os.path.join(drake_distro, "build/drake/doc/Doxyfile_CXX")
    subprocess.check_call(
        [os.path.join(
            _get_drake_distro(), "tools/workspace/cmake_configure_file.py"),
>>>>>>> intial
         "--input", in_filename,
         "--output", doxyfile,
         ] + definition_args)
    assert os.path.exists(doxyfile)

    # Run Doxygen.
    print "Building C++ Doxygen documentation...",
    sys.stdout.flush()
    subprocess.check_call([doxygen, doxyfile], cwd=binary_dir)
<<<<<<< HEAD
    shutil.rmtree(input_root)  # Don't let Bazel find the build/input copy.
=======
>>>>>>> intial
    print "done"
    print "See file://%s/doxygen_cxx/html/index.html" % binary_dir


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.strip())
    parser.add_argument(
        '--quick', action='store_true', default=False,
        help="Disable slow features (e.g., all graphs)")
<<<<<<< HEAD
    parser.add_argument(
        'inputs', nargs='*',
        help="Process only these files and/or directories; "
        "most useful using shell globbing, e.g., "
        "doxygen.py --quick systems/framework/*leaf*.h")
=======
>>>>>>> intial
    args = parser.parse_args()
    _run_doxygen(args)


if __name__ == '__main__':
    main()
