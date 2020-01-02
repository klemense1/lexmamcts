cc_library(
    name = "factories",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/tests:crossing_test_env",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
    ],
)
