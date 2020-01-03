//
// Created by Luis Gressenbuch on 25.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "define_evaluator_rule_ltl.hpp"

#include "ltl_evaluator/evaluator_rule_ltl.h"
#include "ltl_evaluator/evaluator_label_base.h"
#include "ltl_evaluator/label.h"

namespace py = pybind11;
using namespace ltl;
void define_evaluator_rule_ltl(py::module m) {
  py::class_<RuleMonitor, std::shared_ptr<RuleMonitor>>(m, "RuleMonitor")
      .def(py::init(&RuleMonitor::make_rule))
      .def("make_rule", &RuleMonitor::make_rule)
      .def("__repr__", [](const RuleMonitor &m) {
        std::stringstream os;
        os << m;
        return os.str();
      });

  py::class_<Label, std::shared_ptr<Label>>(m, "Label")
  .def(py::init<const std::string&, int>())
  .def(py::init<const std::string&>())
  .def("__repr__", [](const Label &l) {
    std::stringstream os;
    os << l;
    return os.str();
  });
}