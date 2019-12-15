//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#include "zipper_test_env_factory.h"

#include "mcts/mcts_parameters.h"
#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/crossing_state_parameter.h"
#include "test/crossing_test/label_evaluator/evaluator_label_ego_range.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_at_ego_lane_at_pos.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_lane_at.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_range.h"
#include "test/crossing_test/tests/crossing_test_env.h"

const std::string zip_formula =
    "G((mp_oe & X !mp_oe & (w_o | X w_o) & w_e) -> (!mp_e W mp_o)) & G!((mp_oe & "
    "mp_e)|(mp_e & mp_o)|(mp_oe & mp_o)|(w_e & mp_e))";

std::shared_ptr<BaseTestEnv> ZipperTestEnvFactory::make_test_env() {
  MctsParameters mcts_params = make_default_mcts_parameters();
  mcts_params.uct_statistic.LOWER_BOUND << -30.0f, -30.0f, -30.0f, -30.0f,
      -5000.0f;
  mcts_params.uct_statistic.UPPER_BOUND << 0.0f, 0.0f, 0.0f, 0.0f, 5000.0f;
//  mcts_params.thres_uct_statistic_.THRESHOLD << -0.20, -1.0, -1.0, -1.0,
  mcts_params.thres_uct_statistic_.THRESHOLD << -0.66, -0.43, -1.0, -1.0,
      std::numeric_limits<ObjectiveVec::Scalar>::max();
  mcts_params.COOP_FACTOR = 0;
  mcts_params.DISCOUNT_FACTOR = 0.9;
  mcts_params.uct_statistic.EXPLORATION_CONSTANT = 0.7;
  mcts_params.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = std::numeric_limits<double>::infinity();

  CrossingStateParameter crossing_params =
      make_default_crossing_state_parameters();
  crossing_params.depth_prio = 4;
  crossing_params.speed_deviation_prio = 4;
  crossing_params.acceleration_prio = 4;
  crossing_params.potential_prio = 4;

  crossing_params.depth_weight = 0;
  crossing_params.speed_deviation_weight = 2;
  crossing_params.acceleration_weight = 0;
  crossing_params.potential_weight = 0;
  crossing_params.num_other_agents = 2;
  crossing_params.ego_goal_reached_position = 1000;
  crossing_params.state_x_length = 1000;
  crossing_params.crossing_point = 10;
  crossing_params.terminal_depth_ = 25;

  auto automata =
      BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
  for (auto &aut : automata) {
    aut.insert({Rule::ZIP, EvaluatorRuleLTL::make_rule(zip_formula, -1, 1)});
    aut.erase(Rule::NO_SPEEDING);
    aut.erase(Rule::REACH_GOAL);
//    aut.at(Rule::REACH_GOAL)->set_weight(-1.0f);
//    aut.at(Rule::REACH_GOAL)->set_final_reward(0);
    aut.at(Rule::NO_COLLISION)->set_weight(-1.0f);
    aut.erase(Rule::GIVE_WAY);
    aut.erase(Rule::LEAVE_INTERSECTION);
  }
  auto env = std::make_shared<CrossingTestEnv<ThresUCTStatistic>>(
      mcts_params, crossing_params, automata,
      BaseTestEnv::make_default_labels(crossing_params));
  auto agent_states = env->state->get_agent_states();
  agent_states[0].x_pos = crossing_params.crossing_point - 10;
  agent_states[1].x_pos = crossing_params.crossing_point - 1;
  agent_states[2].x_pos = crossing_params.crossing_point - 6;
  agent_states[2].lane = agent_states[1].lane;
  env->label_evaluators_.emplace_back(std::make_shared<EvaluatorLabelEgoRange>(
      "w_e", -5, crossing_params.crossing_point - 1));
  env->label_evaluators_.emplace_back(std::make_shared<EvaluatorLabelOtherRange>(
      "w_o", -5, crossing_params.crossing_point - 1));
  env->label_evaluators_.emplace_back(
      std::make_shared<EvaluatorLabelAtPosition>(
          "mp_e", crossing_params.crossing_point));
  env->label_evaluators_.emplace_back(std::make_shared<EvaluatorLabelOtherAtEgoLaneAtPos>(
      "mp_oe", crossing_params.crossing_point));
  env->label_evaluators_.emplace_back(std::make_shared<EvaluatorLabelOtherLaneAt>(
      "mp_o", crossing_params.crossing_point));

  env->state = std::make_shared<CrossingState>(
      agent_states, false, env->state->get_rule_state_map(),
      env->label_evaluators_, env->crossing_state_parameter_, 0);
  return env;
}
