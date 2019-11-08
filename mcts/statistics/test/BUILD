cc_test(
    name = "pareto_test",
    srcs = [
        "pareto_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@gtest//:main",
    ],
)

cc_test(
    name = "comperator_test",
    srcs = ["comperator_test.cpp"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
    ],
)
