cc_library(
    name = "rule_evaluator",
    srcs = [
        "evaluator_rule_ltl.cpp",
        "label.cpp",
        "rule_state.cpp",
    ],
    hdrs = [
        "common.h",
        "evaluator_label_base.h",
        "evaluator_rule_ltl.h",
        "label.h",
        "rule_state.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
        "@spot",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.h"]),
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
        "@spot",
    ],
)

cc_test(
    name = "automaton_test",
    srcs = ["test/automaton_test.cpp"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":rule_evaluator",
        "@com_github_gflags_gflags//:gflags",
        "@gtest//:main",
    ],
)

cc_test(
    name = "zipper_merge_formula_test",
    srcs = ["test/zipper_merge_formula_test.cpp"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        ":rule_evaluator",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@gtest//:main",
    ],
)
