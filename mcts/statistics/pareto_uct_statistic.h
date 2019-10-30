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
  ParetoUCTStatistic(ActionIdx num_actions) : UctStatistic<ParetoUCTStatistic>(num_actions) {};

  template<class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    if (unexpanded_actions.empty()) {
      // Select an action based on the UCB formula
      std::vector<Eigen::VectorXf> values;
      calculate_ucb_values(ucb_statistics_, values);
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
};

}

#endif //MAMCTS_MCTS_STATISTICS_UCT_STATISTIC_H_PARETOUCT_H_
