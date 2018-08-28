# -*- python -*-

<<<<<<< HEAD
load("@drake//tools/workspace:which.bzl", "which_repository")

def protoc_repository(name):
    # Find the protoc binary.
    which_repository(
=======
load("@drake//tools/workspace:which.bzl", "which")

def protoc_repository(name):
    # Find the protoc binary on $PATH.
    which(
>>>>>>> intial
        name = name,
        command = "protoc",
    )
