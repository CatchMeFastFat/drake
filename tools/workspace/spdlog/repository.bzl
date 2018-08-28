# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def spdlog_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
<<<<<<< HEAD
        commit = "v1.0.0",
        sha256 = "90d5365121bcd2c41ce94dfe6a460e89507a2dfef6133fe5fad5bb35ac4ef0a1",  # noqa
=======
        # This the commit immediately following v0.16.3 that fixes the version
        # badging.  We should change back to saying "v0.17.1" or whatever here
        # (instead of a commit hash) upon the next upstream release.
        commit = "f258af4364ed2aa966ddce8292b9bbde8bbb6152",
        sha256 = "69799a0963fe396e569bedcc9263511a61e3b6dc586bd800bd9597ad3c2268f0",  # noqa
>>>>>>> intial
        build_file = "@drake//tools/workspace/spdlog:package.BUILD.bazel",
        mirrors = mirrors,
    )
