//
// Created by luis on 21.10.19.
//

#ifndef MAMCTS_MCTS_STATISTICS_PUCT_STATISTIC_H_
#define MAMCTS_MCTS_STATISTICS_PUCT_STATISTIC_H_

#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1
#import <cmath>

#include "mcts/statistics/uct_statistic.h"

namespace mcts {

class PuctStatistic : public UctStatistic {

 private:

  double binom_pdf(int k) const {
    int binom = 1 / ((n_ + 1) * std::beta(n_ - k + 1, k + 1));
    return binom * std::pow(p_, k) * std::pow(1 - p_, n_ - k);
  }

  void calculate_ucb_values(const ActionUCBMap &ucb_statistics, std::vector <Eigen::VectorXf> &values) const {
    values.resize(ucb_statistics.size());

    for (size_t idx = 0; idx < ucb_statistics.size(); ++idx) {
      Eigen::VectorXf action_value_normalized =
          (ucb_statistics.at(idx).action_value_ - lower_bound).cwiseQuotient(upper_bound - lower_bound);
      //MCTS_EXPECT_TRUE(action_value_normalized >= 0);
      //MCTS_EXPECT_TRUE(action_value_normalized <= 1);
      values[idx] = action_value_normalized.array()
          + 2 * k_exploration_constant * binom_pdf(idx) * sqrt(total_node_visits_)
              / (1 + ucb_statistics.at(idx).action_count_));
    }
  }

 public:
  template<class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      // Select an action based on the UCB formula
      std::vector <Eigen::VectorXf> values;
      calculate_puct_values(ucb_statistics_, values);
      // find largest index
      ActionIdx selected_action = std::distance(values.begin(),
                                                std::max_element(values.begin(),
                                                                 values.end(),
                                                                 [](Eigen::VectorXf &a, Eigen::VectorXf &b) -> bool {
                                                                   return std::lexicographical_compare(a.begin(),
                                                                                                       a.end(),
                                                                                                       b.begin(),
                                                                                                       b.end());
                                                                 }));
      return selected_action;

    } else {
      // Select randomly an unexpanded action
      std::vector<double> prior(unexpanded_actions.size());
      std::transform(unexpanded_actions.begin(), unexpanded_actions.end(), prior.begin(), &binom_pdf);
      int array_idx = std::distance(prior.begin(), std::max_element(prior.begin(), prior.end()));
      ActionIdx selected_action = unexpanded_actions[array_idx];
      unexpanded_actions.erase(unexpanded_actions.begin() + array_idx);
      return selected_action;
    }
  }

 private:
  double const p_ = 0.25;
  int const n_ = 2;
};

}

#endif //MAMCTS_MCTS_STATISTICS_PUCT_STATISTIC_H_
