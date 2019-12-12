//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_BASE_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_BASE_HPP_

#include <string>
#include <utility>
#include "label.h"
template<class S>
class EvaluatorLabelBase {
 public:
  explicit EvaluatorLabelBase(const std::string& label_str) : label_(label_str) {}
  explicit EvaluatorLabelBase(ltl::Label label) : label_(std::move(label)) {}
  const ltl::Label &get_label() const { return label_; }
  virtual bool evaluate(const S &state) const = 0;
 private:
  ltl::Label label_;
};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_BASE_HPP_
