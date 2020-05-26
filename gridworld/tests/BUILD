cc_library(
    name = "test_runner",
    srcs = ["test_runner.cpp"],
    hdrs = ["test_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//evaluation",
        "//mcts",
        "//gridworld/tests:grid_world_env",
        "//gridworld:grid_world_state",
        "@boost//:math",
        "@boost//:multiprecision",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "grid_world_env",
    srcs = [
        "base_test_env.cpp",
    ],
    hdrs = [
        "base_test_env.h",
        "grid_world_env.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//mcts",
        "//gridworld:grid_world_state",
        "@com_github_eigen_eigen//:eigen",
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
        "//mcts",
        "//gridworld:grid_world_state",
        "//gridworld:state_file_writer",
        "@com_github_google_glog//:glog",
    ],
)
