cc_library(
    name = "mvmcts",
    srcs = [
        "mvmcts_parameters.cpp",
        "random_generator.cpp",
    ],
    hdrs = glob(["**/*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "@boost//:container_hash",
        "@boost//:math",
        "@boost//:multiprecision",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)
