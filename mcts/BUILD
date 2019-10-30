cc_library(
    name = "mcts",
    srcs = [
        "mcts_parameters.cpp",
        "random_generator.cpp",
    ],
    hdrs = glob(["**/*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "@boost//:math",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)
