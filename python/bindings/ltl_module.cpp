//
// Created by Luis Gressenbuch on 25.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "python/bindings/define_evaluator_rule_ltl.hpp"

namespace py = pybind11;

PYBIND11_MODULE(ltl, m) {
  define_evaluator_rule_ltl(m);
}