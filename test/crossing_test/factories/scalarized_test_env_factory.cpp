//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "scalarized_test_env_factory.h"

std::shared_ptr<BaseTestEnv> ScalarizedTestEnvFactory::make_test_env() {
  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params = make_default_crossing_state_parameters();
  crossing_params.speed_deviation_prio = 0;
  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 200;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 0;

  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.DISCOUNT_FACTOR = 0.9;

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.erase(Rule::NO_SPEEDING);
    aut.erase(Rule::REACH_GOAL);
    aut.at(Rule::GIVE_WAY)->set_weight(0.0f);
    aut.at(Rule::NO_COLLISION)->set_weight(-1000.0f);
    aut.at(Rule::LEAVE_INTERSECTION)->set_weight(-300.0f);
  }
  automata[0].at(Rule::GIVE_WAY)->set_weight(-500.0f);
  automata[0].at(Rule::GIVE_WAY)->set_priority(RewardPriority::SAFETY);

  return std::make_shared<CrossingTestEnv<mcts::UctStatistic<>>>(
      mcts_params, crossing_params, automata);
}
