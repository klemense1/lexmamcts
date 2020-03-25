cc_library(
    name = "util",
    srcs = [
        "util.cpp",
    ],
    hdrs = [
        "crossing_state_episode_runner.h",
        "util.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "test_runner",
    srcs = ["test_runner.cpp"],
    hdrs = ["test_runner.h"],
    deps = [
        "//evaluation",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/factories",
        "@boost//:math",
        "@boost//:multiprecision",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "crossing_test_env",
    srcs = [
        "base_test_env.cpp",
    ],
    hdrs = [
        "base_test_env.h",
        "crossing_test_env.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "crossing_state_episode_runner",
    hdrs = ["crossing_state_episode_runner.h"],
    deps = [":crossing_test_env"],
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
        ":crossing_test_env",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/factories",
        "@com_github_gflags_gflags//:gflags",
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
        ":crossing_test_env",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "trajectory_compare",
    srcs = [
        "trajectory_compare.cpp",
    ],
    deps = [
        ":test_runner",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "testbench",
    srcs = [
        "testbench.cpp",
    ],
    deps = [
        ":test_runner",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test:state_file_writer",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "threshold_comparison",
    srcs = [
        "threshold_comparison.cpp",
    ],
    deps = [
        ":test_runner",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test:state_file_writer",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "zipper_merge",
    srcs = [
        "zipper_merge_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":crossing_test_env",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test:state_file_writer",
        "//test/crossing_test/factories",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "coop_factor",
    srcs = [
        "coop_factor_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":crossing_test_env",
        ":test_runner",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test:state_file_writer",
        "//test/crossing_test/factories",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
    ],
)

cc_test(
    name = "minimum_violation",
    srcs = ["minimum_violation_test.cpp"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":crossing_test_env",
        ":test_runner",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "performance",
    srcs = [
        "performance.cpp",
    ],
    deps = [
        ":test_runner",
        ":util",
        "//evaluation:evaluation",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "pareto_convergence",
    srcs = [
        "pareto_convergence.cpp",
    ],
    deps = [
        ":test_runner",
        ":util",
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test:state_file_writer",
        "//test/crossing_test/factories",
        "@com_github_google_glog//:glog",
    ],
)