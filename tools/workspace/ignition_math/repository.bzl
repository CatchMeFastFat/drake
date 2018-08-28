# -*- python -*-

load("//tools/workspace:bitbucket.bzl", "bitbucket_archive")

def ignition_math_repository(
        name,
        mirrors = None):
<<<<<<< HEAD
    # When updating this commit, also remember to adjust the PROJECT_*
    # constants in ./package.BUILD.bazel to match the new version number.
    commit = "8fa77870d791"
=======
    commit = "568cf1457760"
>>>>>>> intial
    bitbucket_archive(
        name = name,
        repository = "ignitionrobotics/ign-math",
        commit = commit,
<<<<<<< HEAD
        sha256 = "09c538cc302f9c50fa7fd6c1a6e367b92c1c9f779f8b4978e0d23e40bfbecb2b",  # noqa
=======
        sha256 = "bd0cafb01cc219c5e12c6b049cae44cfa68bb39bdb52a3c79c37f2163d7d4967",  # noqa
>>>>>>> intial
        strip_prefix = "ignitionrobotics-ign-math-%s" % (commit),
        build_file = "@drake//tools/workspace/ignition_math:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
