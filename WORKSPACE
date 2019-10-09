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
