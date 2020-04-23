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

cc_binary(
    name = "convergence_comparison",
    srcs = [
        "convergence_comparison.cpp",
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
    name = "performance",
    srcs = [
        "performance.cpp",
    ],
    deps = [
        ":test_runner",
        ":util",
        "//evaluation",
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
