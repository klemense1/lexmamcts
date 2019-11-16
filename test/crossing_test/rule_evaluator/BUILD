cc_library(
    name = "rule_evaluator",
    srcs = [
        "evaluator_rule_ltl.cpp",
        "rule_state.cpp",
    ],
    hdrs = [
        "common.h",
        "evaluator_rule_ltl.h",
        "rule_state.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//test/crossing_test/label_evaluator:label_evaluators",
        "@spot",
    ],
)

cc_test(
    name = "automaton_test",
    srcs = ["automaton_test.cpp"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":rule_evaluator",
        "@gtest//:main",
    ],
)
