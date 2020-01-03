//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "minimum_violation_test_env_factory.h"
#include "mcts/statistics/thres_uct_statistic.h"

std::shared_ptr<BaseTestEnv> MinimumViolationTestEnvFactory::make_test_env() {
  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.uct_statistic.LOWER_BOUND << -30.0f, -30.0f, -30.0f, -30.0f,
      -5000.0f;
  mcts_params.uct_statistic.UPPER_BOUND << 0.0f, 0.0f, 0.0f, 0.0f, 5000.0f;
  mcts_params.thres_uct_statistic_.THRESHOLD << -0.43, -0.59, -0.1, -0.99,
      std::numeric_limits<ObjectiveVec::Scalar>::max();

  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params.ego_goal_reached_position = 15;
  crossing_params.depth_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.speed_deviation_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.acceleration_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.potential_prio = static_cast<int>(RewardPriority::GOAL);
  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 0;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 1;

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.at(Rule::NO_SPEEDING)->set_weight(-1.0f);
    aut.at(Rule::NO_SPEEDING)->set_priority(RewardPriority::LEGAL_RULE_C);
    aut.at(Rule::REACH_GOAL)->set_weight(-100.0f);
    aut.at(Rule::REACH_GOAL)->set_final_reward(100.0f);
    aut.at(Rule::GIVE_WAY)->set_weight(0.0f);
    aut.at(Rule::NO_COLLISION)->set_weight(-1.0f);
    aut.erase(Rule::LEAVE_INTERSECTION);
  }
  automata[0].at(Rule::GIVE_WAY)->set_weight(-1.0f);
  automata[0].insert(
      {Rule::REACH_GOAL_FIRST,
       RuleMonitor::make_rule("!other_goal_reached U goal_reached", -1.f,
                              RewardPriority::LEGAL_RULE_B)});

  auto test_env_ptr =
      std::make_shared<CrossingTestEnv<mcts::ThresUCTStatistic>>(
          mcts_params, crossing_params, automata,
          BaseTestEnv::make_default_labels(crossing_params));
  return std::dynamic_pointer_cast<BaseTestEnv>(test_env_ptr);
}