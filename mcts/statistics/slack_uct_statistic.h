//
// Created by Luis Gressenbuch on 31.10.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
#define MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_

#include <string>
#include <utility>
#include <vector>

#include "boost/math/distributions/students_t.hpp"
#include "glog/logging.h"

#include "mcts/statistics/uct_statistic.h"

namespace mcts {

class SlackUCTStatistic : public UctStatistic<SlackUCTStatistic> {
  typedef NodeStatistic<SlackUCTStatistic> ParentType;

 public:
  template <class T>
  struct SlackComparator {
    explicit SlackComparator(T slack) : slack_(std::move(slack)) {}
    bool operator()(const T &a, const T &b) const {
      assert(a.rows() == b.rows() && a.rows() == slack_.rows() && b.rows() == slack_.rows());
      auto a_upper = a + slack_;
      auto a_lower = a - slack_;
      auto b_upper = b + slack_;
      auto b_lower = b - slack_;
      for (long i = 0; i < a_upper.rows(); ++i) {
        if (a_upper(i) < b_lower(i)) {
          return true;
        } else if (b_upper(i) < a_lower(i)) {
          return false;
        }
      }
      return false;
    }
    const T slack_;
  };

  SlackUCTStatistic(ActionIdx num_actions,
                    MctsParameters const &mcts_parameters)
      : UctStatistic<SlackUCTStatistic>(num_actions, mcts_parameters) {}

  ActionIdx get_best_action() {
    // Lexicographical ordering of the UCT value vectors
    ObjectiveVec slack;
    std::vector<ObjectiveVec> qval;
    std::transform(ucb_statistics_.begin(), ucb_statistics_.end(), std::back_inserter(qval),
                   [](const ActionUCBMap::value_type &elem) { return elem.second.action_value_; });
    calculate_slack_values(qval, &slack);
    SlackComparator<ObjectiveVec> comp(slack);
    VLOG(1) << "Slack: " << slack;
    auto max = std::max_element(ucb_statistics_.begin(), ucb_statistics_.end(),
                                [comp](ActionUCBMap::value_type const &a,
                                       ActionUCBMap::value_type const &b) {
                                  if (a.second.action_count_ == 0) {
                                    return true;
                                  } else if (b.second.action_count_ == 0) {
                                    return false;
                                  } else {
                                    return comp(a.second.action_value_, b.second.action_value_);
                                  }
                                });
    return max->first;
  }

  template <class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      std::uniform_real_distribution<double> uniform_norm(0.0, 1.0);
      std::vector<Eigen::VectorXd> values;
      Eigen::VectorXd slack;
      calculate_ucb_values(ucb_statistics_, values);
      calculate_slack_values(values, &slack);
      ActionIdx selected_action = std::distance(
          values.begin(), std::max_element(values.begin(), values.end(), SlackComparator<Eigen::VectorXd>(slack)));
      const double epsilon = mcts_parameters_.e_greedy_uct_statistic_.EPSILON;
      if (uniform_norm(random_generator_) <= 1.0 - epsilon + epsilon / num_actions_) {
        // Select an action based on the UCB formula
        return selected_action;
      } else {
        std::uniform_int_distribution<ActionIdx> uniform_action(0, num_actions_ - 2);
        ActionIdx random_action = uniform_action(random_generator_);
        return (random_action == selected_action ? random_action + 1 : random_action);
      }
    } else {
      // Select randomly an unexpanded action
      std::uniform_int_distribution<ActionIdx> random_action_selection(0, unexpanded_actions.size() - 1);
      ActionIdx array_idx = random_action_selection(random_generator_);
      ActionIdx selected_action = unexpanded_actions[array_idx];
      unexpanded_actions.erase(unexpanded_actions.begin() + array_idx);
      return selected_action;
    }
  }

 protected:
  template <class T>
  void calculate_slack_values(const std::vector<T> &qval, T *values) const {
    values->resize(qval.size());
    *values = T::Constant(mcts_parameters_.REWARD_VEC_SIZE, -std::numeric_limits<typename T::Scalar>::infinity());
    for (ActionIdx idx = 0; idx < qval.size(); ++idx) {
      *values = values->cwiseMax(qval.at(idx));
    }
    // After C. Li and K. Czarnecki, “Urban Driving with Multi-Objective Deep Reinforcement Learning,” arXiv:1811.08586
    // [cs], Nov. 2018.
    *values = 0.2 * values->cwiseAbs();
  }
};
}  // namespace mcts
#endif  // MCTS_STATISTICS_SLACK_UCT_STATISTIC_H_
