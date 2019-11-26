package(default_visibility = ["//visibility:public"])

# the following rules are only for testing purposes
cc_binary(
    name = "mamcts.so",
    srcs = [
        "common.hpp",
        "define_environments.cpp",
        "define_environments.hpp",
        "define_mamcts.cpp",
        "define_mamcts.hpp",
        "module.cpp",
    ],
    linkshared = 1,
    linkstatic = 1,
    deps = [
        "//mcts",
        "//test/crossing_test:crossing_state",
        "//test/crossing_test/tests:util",
        "@com_github_eigen_eigen//:eigen",
        "@pybind11",
        "@python_linux//:python-lib",
        "@spot",
    ],
)

cc_binary(
    name = "ltl.so",
    srcs = [
        "common.hpp",
        "define_evaluator_rule_ltl.cpp",
        "define_evaluator_rule_ltl.hpp",
        "ltl_module.cpp",
    ],
    linkshared = 1,
    linkstatic = 1,
    deps = [
        "//ltl_evaluator:rule_evaluator",
        "@pybind11",
        "@python_linux//:python-lib",
        "@spot",
    ],
)

py_test(
    name = "python_import_test",
    srcs = ["python_import_test.py"],
    data = ["//python/bindings:mamcts.so"],
    imports = ["../bindings"],
)
