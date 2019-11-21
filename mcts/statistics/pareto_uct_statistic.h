//
// Created by luis on 30.10.19.
//

#ifndef MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_
#define MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_

#include "uct_statistic.h"
#include "mcts/statistics/pareto_set.h"
namespace mcts {

class ParetoUCTStatistic : public UctStatistic<ParetoUCTStatistic> {

 public:
  ParetoUCTStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<ParetoUCTStatistic>(
      num_actions,
      mcts_parameters) {};

  template<class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      // Select an action based on the UCB formula
      std::vector<Eigen::VectorXf> values;
      calculate_modified_ucb_values(ucb_statistics_, values);
      ParetoSet<ActionIdx, Eigen::VectorXf> pareto_set;
      pareto_set.add(values);
      ActionIdx selected_action = pareto_set.get_random();
      return selected_action;
    } else {
      // Select randomly an unexpanded action
      std::uniform_int_distribution<ActionIdx> random_action_selection(0, unexpanded_actions.size() - 1);
      ActionIdx array_idx = random_action_selection(random_generator_);
      ActionIdx selected_action = unexpanded_actions[array_idx];
      unexpanded_actions.erase(unexpanded_actions.begin() + array_idx);
      return selected_action;
    }
  }

  void calculate_modified_ucb_values(const ActionUCBMap &ucb_statistics, std::vector<Eigen::VectorXf> &values) const {
    values.resize(ucb_statistics.size());

    for (size_t idx = 0; idx < ucb_statistics.size(); ++idx) {
      Eigen::VectorXf
          action_value_normalized =
          (ucb_statistics.at(idx).action_value_ - this->mcts_parameters_.uct_statistic.LOWER_BOUND).cwiseQuotient(
              this->mcts_parameters_.uct_statistic.UPPER_BOUND - this->mcts_parameters_.uct_statistic.LOWER_BOUND);
      //MCTS_EXPECT_TRUE(action_value_normalized >= 0);
      //MCTS_EXPECT_TRUE(action_value_normalized <= 1);
      values[idx] = action_value_normalized.array()
          + sqrt((4 * log(total_node_visits_) + log(this->mcts_parameters_.REWARD_VEC_SIZE)) / (2 * ucb_statistics.at(idx).action_count_));
    }
  }
};

};

#endif //MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_
