cc_library(
    name = "util",
    srcs = [
        "common.cpp",
        "test_env_factory.cpp",
        "test_runner.cpp",
    ],
    hdrs = [
        "common.h",
        "crossing_state_episode_runner.h",
        "crossing_test_env.h",
        "test_env_factory.h",
        "test_runner.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)

py_test(
    name = "py_crossing_state_test",
    srcs = ["py_crossing_state_test.py"],
    data = ["//python/bindings:mamcts.so"],
    imports = ["../../python/bindings"],
    deps = [
        "//test/crossing_test:pyviewer",
    ],
)

cc_test(
    name = "crossing_state_test",
    srcs = [
        "crossing_state_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
        "@spot",
    ],
)

cc_binary(
    name = "crossing_state_binary",
    srcs = [
        "crossing_state_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
        "@spot",
    ],
)

cc_binary(
    name = "full_expand",
    srcs = [
        "full_expand.cpp",
    ],
    deps = [
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "trajectory_compare",
    srcs = [
        "trajectory_compare.cpp",
    ],
    deps = [
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "minimum_violation",
    srcs = ["minimum_violation_test.cpp"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
    ],
)
