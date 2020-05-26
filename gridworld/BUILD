cc_library(
    name = "common",
    srcs = ["common.cpp"],
    hdrs = ["common.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//mcts",
    ],
)

cc_library(
    name = "state_file_writer",
    srcs = ["state_file_writer.cpp"],
    hdrs = ["state_file_writer.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
    ],
)

cc_library(
    name = "grid_world_state",
    srcs = [
        "grid_world_state.cpp",
    ],
    hdrs = [
        "grid_world_state.hpp",
        "grid_world_state_parameter.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        "@rule_monitor_project//ltl:rule_monitor",
        "//mcts",
        "//gridworld/label_evaluator:label_evaluators",
        "@com_github_google_glog//:glog",
    ],
)
