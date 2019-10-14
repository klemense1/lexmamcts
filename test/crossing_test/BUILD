filegroup(
    name = "crossing_state",
    srcs = [
        "common.hpp",
        "crossing_state.hpp",
        "crossing_state_test.cpp",
        "evaluator_label_base.hpp",
        "evaluator_label_collision.hpp",
        "evaluator_label_goal_reached.hpp",
        "evaluator_rule_ltl.cpp",
        "evaluator_rule_ltl.hpp",
    ],
)

cc_test(
    name = "crossing_state_test",
    srcs = [
        "crossing_state",
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
        "crossing_state",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//mcts",
        "@gtest//:main",
        "@spot",
    ],
)
