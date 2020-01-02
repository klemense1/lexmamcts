// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef LTL_EVALUATOR_EVALUATOR_LABEL_BASE_H_
#define LTL_EVALUATOR_EVALUATOR_LABEL_BASE_H_

#include <string>
#include <utility>
#include "ltl_evaluator/label.h"
template <class S>
class EvaluatorLabelBase {
 public:
  explicit EvaluatorLabelBase(const std::string &label_str)
      : label_(label_str) {}
  explicit EvaluatorLabelBase(ltl::Label label) : label_(std::move(label)) {}
  const ltl::Label &get_label() const { return label_; }
  virtual bool evaluate(const S &state) const = 0;

 private:
  ltl::Label label_;
};

#endif  // LTL_EVALUATOR_EVALUATOR_LABEL_BASE_H_
