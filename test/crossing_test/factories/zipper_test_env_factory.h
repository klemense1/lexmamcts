//
// Created by Luis Gressenbuch on 07.12.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_TESTS_ZIPPER_TEST_ENV_FACTORY_H_
#define TEST_CROSSING_TEST_TESTS_ZIPPER_TEST_ENV_FACTORY_H_

#include "test/crossing_test/factories/test_env_factory.h"

#include "mcts/mcts_parameters.h"
#include "mcts/statistics/thres_uct_statistic.h"
#include "test/crossing_test/crossing_state_parameter.h"
#include "test/crossing_test/label_evaluator/evaluator_label_ego_range.h"
#include "test/crossing_test/label_evaluator/evaluator_label_in_direct_front.h"
#include "test/crossing_test/label_evaluator/evaluator_label_in_range.h"
#include "test/crossing_test/label_evaluator/evaluator_label_on_ego_lane.h"
#include "test/crossing_test/tests/crossing_test_env.h"

template <class Stat>
class ZipperTestEnvFactory : public ITestEnvFactory {
 public:
  std::shared_ptr<BaseTestEnv> make_test_env() override {
    const std::string zip_formula = "(in_direct_front_x#0 & !merged_e & (in_direct_front_x#0 | merged_x#0) U merged_e) -> G(merged_e & merged_x#0 -> !in_direct_front_x#0)";
//        "G((!merged_e & !merged_x#0) -> in_direct_front_x#0) -> G(merged_e -> !in_direct_front_x#0)";

    MctsParameters mcts_params = make_default_mcts_parameters();
    mcts_params.uct_statistic.LOWER_BOUND << -1.0f, -1.0f, -5000.0f;
    mcts_params.uct_statistic.UPPER_BOUND << 0.0f, 0.0f, 5000.0f;
    mcts_params.thres_uct_statistic_.THRESHOLD << -0.81, -0.3, std::numeric_limits<ObjectiveVec::Scalar>::max();
    mcts_params.COOP_FACTOR = 0.0;
    mcts_params.DISCOUNT_FACTOR = 0.9;
    mcts_params.uct_statistic.EXPLORATION_CONSTANT = 0.7;
    mcts_params.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC = std::numeric_limits<double>::infinity();
    mcts_params.e_greedy_uct_statistic_.EPSILON = 0.2;

    CrossingStateParameter crossing_params = make_default_crossing_state_parameters();
    crossing_params.num_other_agents = 2;
    crossing_params.ego_goal_reached_position = 1000;
    crossing_params.state_x_length = 1000;
    crossing_params.crossing_point = 8;
    crossing_params.terminal_depth_ = 25;
    crossing_params.merge = true;
//    crossing_params.action_map.erase(crossing_params.action_map.begin() + 3);

    auto automata = BaseTestEnv::make_default_automata(crossing_params.num_other_agents + 1);
    for (auto &aut : automata) {
      aut.insert({Rule::ZIP, RuleMonitor::make_rule(zip_formula, -1, 1)});
    }
    auto labels = BaseTestEnv::make_default_labels(crossing_params);
    labels.clear();
    labels.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision", crossing_params.crossing_point));
    labels.emplace_back(std::make_shared<EvaluatorLabelEgoRange>("merged_e", crossing_params.crossing_point,
                                                                 crossing_params.state_x_length));
    labels.emplace_back(std::make_shared<EvaluatorLabelInRange>("merged_x", crossing_params.crossing_point,
                                                                crossing_params.state_x_length));
    //  env->label_evaluators_.emplace_back(
    //      std::make_shared<EvaluatorLabelOnEgoLane>("on_ego_lane_x"));
    labels.emplace_back(std::make_shared<EvaluatorLabelInDirectFront>("in_direct_front_x"));

    auto env = std::make_shared<CrossingTestEnv<Stat>>(mcts_params, crossing_params, automata, labels);
    auto agent_states = env->state->get_agent_states();
    agent_states[0].id = 0;
    agent_states[1].id = 1;
    agent_states[2].id = 2;
    agent_states[0].x_pos = crossing_params.crossing_point - 5;
    agent_states[1].x_pos = crossing_params.crossing_point - 3;
    agent_states[2].x_pos = crossing_params.crossing_point - 8;
    agent_states[1].lane = agent_states[0].lane;
    agent_states[1].init_lane = agent_states[0].init_lane;

    env->state = std::make_shared<CrossingState>(agent_states, false, env->state->get_rule_state_map(),
                                                 env->label_evaluators_, env->crossing_state_parameter_, 0,
                                                 std::vector<bool>(agent_states.size(), false));
    return env;
  };
};

#endif  // TEST_CROSSING_TEST_TESTS_ZIPPER_TEST_ENV_FACTORY_H_
