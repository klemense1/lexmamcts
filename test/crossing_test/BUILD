cc_library(
    name = "crossing_state",
    srcs = [
        "crossing_state.cpp",
        "evaluator_rule_ltl.cpp",
    ],
    hdrs = [
        "common.hpp",
        "crossing_state.hpp",
        "crossing_state_episode_runner.h",
        "evaluator_label_base.hpp",
        "evaluator_label_collision.hpp",
        "evaluator_label_goal_reached.hpp",
        "evaluator_label_hold_at_xing.hpp",
        "evaluator_label_other_near.hpp",
        "evaluator_rule_ltl.hpp",
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
