//
// Created by Luis Gressenbuch on 04.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_MCTS_STATISTICS_E_GREEDY_UCT_STATISTIC_H_
#define MAMCTS_MCTS_STATISTICS_E_GREEDY_UCT_STATISTIC_H_

#include <random>
#include <vector>

#include "uct_statistic.h"
#include "mcts/mcts_parameters.h"

namespace mcts {
class EGreedyUCTStatistic : public UctStatistic<EGreedyUCTStatistic> {

 public:

  EGreedyUCTStatistic(ActionIdx num_actions, MctsParameters const &mcts_parameters) : UctStatistic<EGreedyUCTStatistic>(
      num_actions,
      mcts_parameters) {};

  template<class S>
  ActionIdx choose_next_action(const S &state, std::vector<int> &unexpanded_actions) {
    bool fully_expanded = unexpanded_actions.empty();
    ActionIdx selected_action = UctStatistic<EGreedyUCTStatistic>::choose_next_action(state, unexpanded_actions);
    if (fully_expanded) {
      std::uniform_real_distribution<float> uniform_norm(0, 1);
      if (uniform_norm(random_generator_) >= mcts_parameters_.e_greedy_uct_statistic_.EPSILON) {
        return selected_action;
      } else {
        std::uniform_int_distribution<ActionIdx> uniform_action(0, num_actions_ - 2);
        ActionIdx random_action = uniform_action(random_generator_);
        return random_action == selected_action ? random_action + 1 : random_action;
      }
    } else {
      return selected_action;
    }
  }
};
}

#endif  // MAMCTS_MCTS_STATISTICS_E_GREEDY_UCT_STATISTIC_H_
