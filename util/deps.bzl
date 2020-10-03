load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")


def lexmamcts_dependencies():

    _maybe(
        http_archive,
        name = "gtest",
        url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
        sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
        build_file = "@//util:gtest.BUILD",
        strip_prefix = "googletest-release-1.7.0",
    )

    _maybe(
      git_repository,
      name = "com_github_gflags_gflags",
      commit = "addd749114fab4f24b7ea1e0f2f837584389e52c",
      remote = "https://github.com/gflags/gflags"
    )

    _maybe(
      git_repository,
      name = "com_github_google_glog",
      commit = "3ba8976592274bc1f907c402ce22558011d6fc5e",
      remote = "https://github.com/google/glog"
    )

    _maybe(
        http_archive,
        # Need Eigen 3.4 (which is in development) for STL-compatible iterators
        name = "com_github_eigen_eigen",
        build_file = "@//tools:eigen.BUILD",
        sha256 = "4b1120abc5d4a63620a886dcc5d7a7a27bf5b048c8c74ac57521dd27845b1d9f",
        strip_prefix = "eigen-git-mirror-98e54de5e25aefc6b984c168fb3009868a93e217",
        urls = [
            "https://github.com/eigenteam/eigen-git-mirror/archive/98e54de5e25aefc6b984c168fb3009868a93e217.zip",
        ],
    )

    _maybe(
        git_repository,
        name = "com_github_nelhage_rules_boost",
        commit = "8a084196b14a396b6d4ff7c928ffbb6621f0d32c",
        remote = "https://github.com/patrickhart/rules_boost",
        shallow_since = "1549788893 +0100",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)