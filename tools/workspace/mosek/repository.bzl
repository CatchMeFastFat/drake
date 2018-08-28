# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a MOSEK archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")  # noqa
        mosek_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:mosek"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

<<<<<<< HEAD
load("@drake//tools/workspace:execute.bzl", "which")

def _impl(repository_ctx):
    mosek_major_version = 8
    mosek_minor_version = 1
    mosek_patch_version = 0
    mosek_tweak_version = 51

    if repository_ctx.os.name == "mac os x":
        mosek_platform = "osx64x86"
        sha256 = "00aed5ca62acca6689b579503ad19aedb100b4a37dfd6b95e52f52dee414520e"  # noqa
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "ab2f39c1668105acbdfbe6835f59f547dd8b076378d9943ab40839c70e1141a2"  # noqa
    else:
        fail(
            "Operating system is NOT supported",
            attr = repository_ctx.os.name,
        )

    # TODO(jwnimmer-tri) Port to use mirrors.bzl.
    template = "http://download.mosek.com/stable/{}.{}.{}.{}/mosektools{}.tar.bz2"  # noqa
    url = template.format(
        mosek_major_version,
        mosek_minor_version,
        mosek_patch_version,
        mosek_tweak_version,
        mosek_platform,
    )
=======
def _impl(repository_ctx):
    mosek_major_version = 7
    mosek_minor_version = 1

    if repository_ctx.os.name == "mac os x":
        mosek_platform = "osx64x86"
        sha256 = "26c5bc0be667c92d1a5f81d2a1f7694de1fb0a3e9e9064c17f98e425db0a3c64"  # noqa
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "9b2bfcba7bcdd24b7e87ecdcccc11222302ced7b3d2a2af7090bdf625ab7cfae"  # noqa
    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    # TODO(jwnimmer-tri) Port to use mirrors.bzl.
    url = "http://download.mosek.com/stable/{}/mosektools{}.tar.bz2".format(
        mosek_major_version, mosek_platform)
>>>>>>> intial
    root_path = repository_ctx.path("")
    strip_prefix = "mosek/{}".format(mosek_major_version)

    repository_ctx.download_and_extract(
<<<<<<< HEAD
        url,
        root_path,
        sha256 = sha256,
        stripPrefix = strip_prefix,
    )
=======
        url, root_path, sha256 = sha256, stripPrefix = strip_prefix)
>>>>>>> intial

    platform_prefix = "tools/platform/{}".format(mosek_platform)

    if repository_ctx.os.name == "mac os x":
<<<<<<< HEAD
        install_name_tool = which(repository_ctx, "install_name_tool")

        # Note that in the 8.1.0.51 packages, libmosek64.dylib is a copy of
        # libmosek64.8.1.dylib instead of a symlink. Otherwise, the list of
        # files should include the following in place of bin/libmosek64.dylib:
        #
        # "bin/libmosek64.{}.{}.dylib".format(mosek_major_version,
        #                                     mosek_minor_version)
        files = [
            "bin/libcilkrts.5.dylib",
            "bin/libmosek64.dylib",
=======
        install_name_tool = repository_ctx.which("install_name_tool")

        files = [
            "bin/libiomp5.dylib",
            "bin/libmosek64.{}.{}.dylib".format(mosek_major_version,
                                                mosek_minor_version),
>>>>>>> intial
        ]

        for file in files:
            file_path = repository_ctx.path(
<<<<<<< HEAD
                "{}/{}".format(platform_prefix, file),
=======
                "{}/{}".format(platform_prefix, file)
>>>>>>> intial
            )

            result = repository_ctx.execute([
                install_name_tool,
                "-id",
                file_path,
                file_path,
            ])

            if result.return_code != 0:
<<<<<<< HEAD
                fail(
                    "Could NOT change shared library identification name",
                    attr = result.stderr,
                )
=======
                fail("Could NOT change shared library identification name",
                     attr = result.stderr)
>>>>>>> intial

        srcs = []

        bin_path = repository_ctx.path("{}/bin".format(platform_prefix))

        linkopts = [
            "-L{}".format(bin_path),
<<<<<<< HEAD
=======
            "-liomp5",
>>>>>>> intial
            "-lmosek64",
        ]
    else:
        files = [
<<<<<<< HEAD
            # N.B. We are using and installing MOSEK's copy of libcilkrts.so.5,
            # even though Ubuntu installs the same shared library by default on
            # all systems already. For some reason, Mosek fails when used with
            # Ubuntu's shared library. If Drake users have other third-party
            # code that assumes use of Ubunut's libcilkrts, there could be
            # runtime conflicts; however, this risk seems low.
            "bin/libcilkrts.so.5",
            "bin/libiomp5.so",
            "bin/libmosek64.so.{}.{}".format(
                mosek_major_version,
                mosek_minor_version,
            ),
=======
            "bin/libiomp5.so",
            "bin/libmosek64.so.{}.{}".format(mosek_major_version,
                                             mosek_minor_version),
>>>>>>> intial
        ]

        linkopts = []
        srcs = ["{}/{}".format(platform_prefix, file) for file in files]

    hdrs = ["{}/h/mosek.h".format(platform_prefix)]
    includes = ["{}/h".format(platform_prefix)]
    files = ["{}/{}".format(platform_prefix, file) for file in files]
    libraries_strip_prefix = ["{}/bin".format(platform_prefix)]

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by mosek_repository()

load("@drake//tools/install:install.bzl", "install", "install_files")

licenses([
    "by_exception_only",  # MOSEK
    "notice",  # fplib AND Zlib
])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mosek",
    srcs = {},
    hdrs = {},
    includes = {},
    linkopts = {},
)

install_files(
    name = "install_libraries",
    dest = "lib",
    files = {},
    strip_prefix = {},
    visibility = ["//visibility:private"],
)

install(
   name = "install",
<<<<<<< HEAD
   docs = [
       "mosek-eula.pdf",
       "@drake//tools/workspace/mosek:LICENSE_CilkPlus",
       "@drake//tools/workspace/mosek:LICENSE_OpenMP",
   ],
   allowed_externals = [
       "@drake//tools/workspace/mosek:LICENSE_CilkPlus",
       "@drake//tools/workspace/mosek:LICENSE_OpenMP",
   ],
=======
   docs = ["license.pdf"],
>>>>>>> intial
   deps = [":install_libraries"],
)
    """.format(srcs, hdrs, includes, linkopts, files, libraries_strip_prefix)

<<<<<<< HEAD
    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )
=======
    repository_ctx.file("BUILD.bazel", content = file_content,
                        executable = False)
>>>>>>> intial

mosek_repository = repository_rule(implementation = _impl)
