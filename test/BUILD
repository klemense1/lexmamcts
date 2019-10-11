cc_test(
    name = "mcts_test",
    srcs = [
        "mcts_test.cc",
        "mcts_test_class.h",
        "simple_state.h",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@gtest//:main",
    ],
)

cc_library(
    name = "mo_deep_sea",
    srcs = ["mo_deep_sea_state.cpp"],
    hdrs = ["mo_deep_sea_state.hpp"],
    deps = [
        "//mcts",
        "@com_github_eigen_eigen//:eigen",
    ],
)

cc_test(
    name = "momcts_test",
    srcs = [
        "momcts_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":mo_deep_sea",
        "//mcts",
        "@gtest//:main",
    ],
)

cc_test(
    name = "crossing_state_test",
    srcs = [
        "crossing_state.hpp",
        "crossing_state_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@gtest//:main",
    ],
)
