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
    sha256 = "4b1120abc5d4a63620a886dcc5d7a7a27bf5b048c8c74ac57521dd27845b1d9f",
    strip_prefix = "eigen-git-mirror-98e54de5e25aefc6b984c168fb3009868a93e217",
    urls = [
        "https://github.com/eigenteam/eigen-git-mirror/archive/98e54de5e25aefc6b984c168fb3009868a93e217.zip",
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

# External dependency: Google Log; has Bazel build already.
http_archive(
    name = "com_github_google_glog",
    sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
    strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
    urls = [
        "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
    ],
)

# External dependency: Google Flags; has Bazel build already.
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ],
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "8a084196b14a396b6d4ff7c928ffbb6621f0d32c",
    remote = "https://github.com/patrickhart/rules_boost",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()
