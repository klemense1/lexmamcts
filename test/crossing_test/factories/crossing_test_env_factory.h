//
// Created by Luis Gressenbuch on 17.02.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef TEST_CROSSING_TEST_FACTORIES_CROSSING_TEST_ENV_FACTORY_H_
#define TEST_CROSSING_TEST_FACTORIES_CROSSING_TEST_ENV_FACTORY_H_

#include "test/crossing_test/factories/test_env_factory.h"

#include <utility>
template <class Stat>
class CrossingTestEnvFactory : public ITestEnvFactory {
 public:
  explicit CrossingTestEnvFactory(ObjectiveVec thres = make_default_mcts_parameters().thres_uct_statistic_.THRESHOLD,
                                  bool enable_right_of_way = true,
                                  float slack_factor = 0.2f)
      : thres_(std::move(thres)), enable_right_of_way_(enable_right_of_way), slack_factor_(slack_factor) {}
  std::shared_ptr<BaseTestEnv> make_test_env() override {
    auto mcts_params = make_default_mcts_parameters();
    mcts_params.thres_uct_statistic_.THRESHOLD = thres_;
    mcts_params.slack_uct_statistic_.SLACK_FACTOR = slack_factor_;
    auto automata = CrossingTestEnv<Stat>::make_default_automata(2);
    if (enable_right_of_way_) {
      // Ego should give way
      automata[0].insert({Rule::GIVE_WAY, RuleMonitor::make_rule("!crossed W other_crossed", -1.0f, 1)});
    }
    auto env =  std::make_shared<CrossingTestEnv<Stat>>(mcts_params, make_default_crossing_state_parameters(), automata);
    auto agent_states = env->state->get_agent_states();
    agent_states[0].x_pos = 0;
    agent_states[1].x_pos = 1;

    env->state = std::make_shared<CrossingState>(agent_states, false, env->state->get_rule_state_map(),
                                                 env->label_evaluators_, env->crossing_state_parameter_, 0,
                                                 std::vector<bool>(agent_states.size(), false));
    return std::dynamic_pointer_cast<BaseTestEnv>(env);
  }

 private:
  ObjectiveVec thres_;
  bool enable_right_of_way_;
  float slack_factor_;
};

#endif  // TEST_CROSSING_TEST_FACTORIES_CROSSING_TEST_ENV_FACTORY_H_
