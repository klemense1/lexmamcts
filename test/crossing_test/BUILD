cc_test(
    name = "crossing_state_test",
    srcs = [
        "crossing_state.hpp",
        "crossing_state_test.cpp",
        "evaluator_rule_ltl.cpp",
        "evaluator_rule_ltl.hpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@gtest//:main",
        "@spot",
    ],
)

cc_binary(
    name = "crossing_state_binary",
    srcs = [
        "crossing_state.hpp",
        "crossing_state_test.cpp",
        "evaluator_rule_ltl.cpp",
        "evaluator_rule_ltl.hpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@gtest//:main",
        "@spot",
    ],
)
