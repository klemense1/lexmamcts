//
// Created by Luis Gressenbuch on 25.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "define_evaluator_rule_ltl.hpp"

#include "ltl_evaluator/evaluator_rule_ltl.h"
#include "ltl_evaluator/evaluator_label_base.h"

namespace py = pybind11;
using namespace ltl;
void define_evaluator_rule_ltl(py::module m) {
  py::class_<EvaluatorRuleLTL,
             std::shared_ptr<EvaluatorRuleLTL>>(m, "EvaluatorRuleLTL")
      .def(py::init(&EvaluatorRuleLTL::make_rule))
      .def("make_rule", &EvaluatorRuleLTL::make_rule)
      .def("__repr__", [](const EvaluatorRuleLTL &m) {
        return "ltl.EvaluatorRuleLTL";
      });
}