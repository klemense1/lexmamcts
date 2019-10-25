py_test(
    name = "py_crossing_state_test",
    srcs = ["py_crossing_state_test.py"],
    data = ["//python/bindings:mamcts.so"],
    imports = ["../../python/bindings"],
    deps = [
        "//test/crossing_test:pyviewer",
    ],
)

cc_test(
    name = "crossing_state_test",
    srcs = [
        "crossing_state_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
        "@spot",
    ],
)

cc_binary(
    name = "crossing_state_binary",
    srcs = [
        "crossing_state_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "//test/crossing_test:crossing_state",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
        "@spot",
    ],
)
