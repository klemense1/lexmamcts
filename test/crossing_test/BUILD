cc_library(
    name = "crossing_state",
    srcs = [
        "crossing_state.cpp",
        "evaluator_rule_ltl.cpp",
    ],
    hdrs = [
        "common.hpp",
        "crossing_state.hpp",
        "evaluator_rule_ltl.hpp",
        "label_evaluator/evaluator_label_base.hpp",
        "label_evaluator/evaluator_label_collision.hpp",
        "label_evaluator/evaluator_label_goal_reached.hpp",
        "label_evaluator/evaluator_label_hold_at_xing.hpp",
        "label_evaluator/evaluator_label_other_near.hpp",
        "label_evaluator/evaluator_label_speed.hpp",
        "viewer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//mcts",
        "@com_github_google_glog//:glog",
        "@spot",
    ],
)

py_library(
    name = "pyviewer",
    srcs = ["pyviewer.py"],
    data = ["//python/bindings:mamcts.so"],
    imports = ["../../python/bindings"],
    visibility = ["//visibility:public"],
)
