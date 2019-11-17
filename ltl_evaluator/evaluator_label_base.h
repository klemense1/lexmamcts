//
// Created by luis on 14.10.19.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_BASE_HPP_
#define MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_BASE_HPP_

#include <string>
template<class S>
class EvaluatorLabelBase {
 public:
  EvaluatorLabelBase(std::string label_str) : label_str_(label_str) {}
  const std::string &get_label_str() const { return label_str_; }
  virtual bool evaluate(const S &state) const = 0;
 private:
  std::string label_str_;

};

#endif //MAMCTS_TEST_CROSSING_TEST_EVALUATOR_LABEL_BASE_HPP_
