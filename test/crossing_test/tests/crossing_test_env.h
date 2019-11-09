//
// Created by Luis Gressenbuch on 04.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
#define MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_

#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/heuristics/random_heuristic.h"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/viewer.h"
#include "mcts/random_generator.h"
#include "test/crossing_test/label_evaluator/evaluator_label_collision.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_goal_reached.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_hold_at_xing.hpp"
#include "test/crossing_test/label_evaluator/evaluator_label_other_near.hpp"
#include "test/crossing_test/common.hpp"
#include "mcts/statistics/pareto_uct_statistic.h"
#include "mcts/statistics/slack_uct_statistic.h"

namespace mcts {

template<class Stats = UctStatistic<>>
class CrossingTestEnv {
 public:
  CrossingTestEnv(MctsParameters const &mcts_parameters, CrossingStateParameter const &crossing_state_parameter)
      : mcts_parameters_(mcts_parameters),
        crossing_state_parameter_(crossing_state_parameter),
        mcts(mcts_parameters_) {
    // SETUP LABEL EVALUATORS
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision",
                                                                            crossing_state_parameter_.crossing_point));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelGoalReached>("goal_reached",
                                                                              crossing_state_parameter_.ego_goal_reached_position));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelHoldAtXing>("at_hp_xing",
                                                                             crossing_state_parameter_.crossing_point));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
    // SETUP RULES
    automata.resize(crossing_state_parameter_.num_other_agents + 1);

    // Finally arrive at goal (Liveness)
    automata[0].emplace_back("F goal_reached", -100.f, RewardPriority::GOAL, 1.0f);
    // Do not collide with others (Safety)
    automata[0].emplace_back("G !collision", -1000.f, RewardPriority::SAFETY);
    // Copy rules to other agents
    for (size_t i = 1; i < automata.size(); ++i) {
      automata[i] = Automata::value_type(automata[0]);
    }

    // Rules only for ego
    automata[0].emplace_back("G((at_hp_xing & other_near) -> (X at_hp_xing))", -100.0f, RewardPriority::SAFETY);
    // Arrive before others (Guarantee)
    // Currently not possible because ego can't drive faster than others
    //automata.emplace_back("!other_goal_reached U ego_goal_reached", -1000.f, RewardPriority::GOAL);
    for (size_t i = 0; i < automata.size(); i++) {
      LOG(INFO) << "Rules for agent " << i << ":";
      for (auto const &rule : automata[i]) {
        LOG(INFO) << rule;
      }
    }
    state = std::make_shared<CrossingState>(automata, label_evaluators, crossing_state_parameter_);
    rewards = std::vector<Reward>(state->get_agent_idx().size(), Reward::Zero());
    jt = JointAction(2, (int) Actions::FORWARD);
  }
  CrossingTestEnv() : CrossingTestEnv(make_std_mcts_parameters(), make_default_crossing_state_parameters()) {};
  ~CrossingTestEnv() {
    LOG(INFO) << "Ego positions:" << pos_history;
    LOG(INFO) << "Otr positions:" << pos_history_other;
  }
  MctsParameters const mcts_parameters_;
  CrossingStateParameter const crossing_state_parameter_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
  Automata automata;
  std::vector<Reward> rewards;
  JointAction jt;
  std::vector<std::size_t> pos_history;
  std::vector<size_t> pos_history_other;
  Mcts<CrossingState, Stats, Stats, RandomHeuristic> mcts;
  std::shared_ptr<CrossingState> state;
};

}

#endif  // MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
