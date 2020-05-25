cc_library(
    name = "util",
    srcs = [
        "util.cpp",
    ],
    hdrs = [
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
    visibility = ["//visibility:public"],
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
