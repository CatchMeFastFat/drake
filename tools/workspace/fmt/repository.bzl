# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fmt_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "fmtlib/fmt",
<<<<<<< HEAD
        commit = "5.1.0",
        sha256 = "73d4cab4fa8a3482643d8703de4d9522d7a56981c938eca42d929106ff474b44",  # noqa
=======
        commit = "4.1.0",
        sha256 = "46628a2f068d0e33c716be0ed9dcae4370242df135aed663a180b9fd8e36733d",  # noqa
>>>>>>> intial
        build_file = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        mirrors = mirrors,
    )
