//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "max_uct_test_env_factory.h"
#include "mcts/statistics/max_uct_statistic.h"

std::shared_ptr<BaseTestEnv> MaxUCTTestEnvFactory::make_test_env() {
  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params = make_default_crossing_state_parameters();
  crossing_params.depth_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.speed_deviation_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.acceleration_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.potential_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 0;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 1;

  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.uct_statistic.PROGRESSIVE_WIDENING_ALPHA = 0.25;
  mcts_params.uct_statistic.PROGRESSIVE_WIDENING_ENABLED = true;
  mcts_params.DISCOUNT_FACTOR = 1.0;

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.erase(Rule::LEAVE_INTERSECTION);
    aut.erase(Rule::REACH_GOAL);
    aut.at(Rule::NO_SPEEDING)->set_weight(-1.0f);
    aut.at(Rule::GIVE_WAY)->set_weight(0.0f);
    aut.at(Rule::NO_COLLISION)->set_weight(-1.0f);
  }
  automata[0].at(Rule::GIVE_WAY)->set_weight(-1.0f);
  return std::make_shared<CrossingTestEnv<mcts::MaxUCTStatistic>>(
      mcts_params, crossing_params, automata);
}