cc_library(
    name = "mo_deep_sea",
    srcs = ["mo_deep_sea_state.cpp"],
    hdrs = ["mo_deep_sea_state.hpp"],
    deps = [
        "//mcts",
        "@com_github_eigen_eigen//:eigen",
    ],
)

cc_binary(
    name = "mo_deep_sea_test",
    srcs = [
        "mo_deep_sea_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":mo_deep_sea",
        "//mcts",
        "@gtest//:main",
    ],
)
