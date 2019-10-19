load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "gtest",
    build_file = "@//test/external:gtest.BUILD",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    strip_prefix = "googletest-release-1.7.0",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
)

http_archive(
    # Need Eigen 3.4 (which is in development) for STL-compatible iterators
    name = "com_github_eigen_eigen",
    build_file = "@//test/external:eigen.BUILD",
    #sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
    sha256 = "e91cfa2bee47d3dfcd41c8ea9467324e4937b3ca5324abc71caaf5ef30372d7c",
    #strip_prefix = "eigen-eigen-5a0156e40feb",
    strip_prefix = "eigen-eigen-70e55a287bfe",
    urls = [
        #"http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",
        "https://bitbucket.org/eigen/eigen/get/tip.zip",
    ],
)

http_archive(
    name = "spot",
    build_file = "@//test/external:spot.BUILD",
    patch_cmds = ["./configure"],
    sha256 = "dcb7aa684725304afb3d435f26f25b51fbd6e9a6ef610e16163cc0030ad5eab4",
    strip_prefix = "spot-2.8.1",
    urls = ["http://www.lrde.epita.fr/dload/spot/spot-2.8.1.tar.gz"],
)

new_local_repository(
    name = "python_linux",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"],
    visibility = ["//visibility:public"],
)
    """,
    path = "./python/venv/",
)

http_archive(
    name = "pybind11",
    build_file_content = """
cc_library(
    name = "pybind11",
    hdrs = glob([
        "include/**/**/*.h",
    ]),
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include/"
)
""",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
)
