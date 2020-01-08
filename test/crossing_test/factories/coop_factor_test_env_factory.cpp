//
// Created by Luis Gressenbuch on 06.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "coop_factor_test_env_factory.h"
#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/label_evaluator/evaluator_label_ego_range.h"
std::shared_ptr<BaseTestEnv> CoopFactorTestEnvFactory::make_test_env() {
  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.REWARD_VEC_SIZE = 2;
  mcts_params.uct_statistic.LOWER_BOUND =
      mcts::ObjectiveVec::Zero(mcts_params.REWARD_VEC_SIZE);
  mcts_params.uct_statistic.LOWER_BOUND << -30.0f, -5000.0f;
  mcts_params.uct_statistic.UPPER_BOUND =
      mcts::ObjectiveVec::Zero(mcts_params.REWARD_VEC_SIZE);
  mcts_params.uct_statistic.UPPER_BOUND << 0.0f, 5000.0f;
  mcts_params.thres_uct_statistic_.THRESHOLD =
      mcts::ObjectiveVec::Zero(mcts_params.REWARD_VEC_SIZE);
  mcts_params.thres_uct_statistic_.THRESHOLD << -0.25,
      std::numeric_limits<ObjectiveVec::Scalar>::max();
  mcts_params.COOP_FACTOR = coop_factor_;
  mcts_params.DISCOUNT_FACTOR = 0.95;
  mcts_params.uct_statistic.EXPLORATION_CONSTANT = 0.7;
  mcts_params.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC =
      std::numeric_limits<double>::infinity();

  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params.reward_vec_size = mcts_params.REWARD_VEC_SIZE;
  crossing_params.depth_prio = 1;
  crossing_params.speed_deviation_prio = 1;
  crossing_params.acceleration_prio = 1;
  crossing_params.potential_prio = 1;

  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 2;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 0;
  crossing_params.num_other_agents = 1;
  crossing_params.ego_goal_reached_position = 1000;
  crossing_params.state_x_length = 1000;
  crossing_params.crossing_point = 10;
  crossing_params.terminal_depth_ = 25;
  crossing_params.action_map = {static_cast<int>(Actions::FORWARD),
                                static_cast<int>(Actions::WAIT)};

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.erase(Rule::NO_SPEEDING);
    aut.erase(Rule::REACH_GOAL);
    aut.at(Rule::NO_COLLISION)->set_weight(-1.0f);
    aut.erase(Rule::GIVE_WAY);
    aut.erase(Rule::LEAVE_INTERSECTION);
    aut.insert({Rule::OBSTACLE,
                ltl::RuleMonitor::make_rule("G !collision_obstacle", -1.0, 0)});
  }
  auto env = std::make_shared<CrossingTestEnv<ThresUCTStatistic>>(
      mcts_params, crossing_params, automata,
      BaseTestEnv::make_default_labels(crossing_params));
  auto agent_states = env->state->get_agent_states();
  agent_states[0].x_pos = crossing_params.crossing_point - 2;
  agent_states[1].x_pos = crossing_params.crossing_point - 1;
  env->label_evaluators_.emplace_back(std::make_shared<EvaluatorLabelEgoRange>(
      "collision_obstacle", crossing_params.crossing_point + 1,
      crossing_params.state_x_length));
  auto rule_state_map = env->state->get_rule_state_map();
  rule_state_map[0].erase(rule_state_map[0].find(Rule::OBSTACLE));
  rule_state_map[1].erase(rule_state_map[1].find(Rule::NO_COLLISION));

  env->state = std::make_shared<CrossingState>(
      agent_states, false, rule_state_map, env->label_evaluators_,
      env->crossing_state_parameter_, 0,
      std::vector<bool>(agent_states.size(), false));
  return env;
}
CoopFactorTestEnvFactory::CoopFactorTestEnvFactory(const float coop_factor)
    : coop_factor_(coop_factor) {}
