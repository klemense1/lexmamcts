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
#include "test/crossing_test/label_evaluator/evaluator_label_speed.hpp"
#include "common.h"

using namespace mcts;

enum Rule {
  NO_COLLISION = 0, REACH_GOAL, NO_SPEEDING, GIVE_WAY, LEAVE_INTERSECTION, NUM,
};

template<class Stats = UctStatistic<>, class Heuristic = RandomHeuristic>
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
                                                                             crossing_state_parameter_.crossing_point+1));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
    label_evaluators.emplace_back(std::make_shared<EvaluatorLabelSpeed>("speeding"));
    // SETUP RULES
    automata.resize(crossing_state_parameter_.num_other_agents + 1);

    automata[0].insert({Rule::NO_SPEEDING, EvaluatorRuleLTL("G !speeding", -1.0f, RewardPriority::LEGAL_RULE_B)});
    automata[0].insert({Rule::REACH_GOAL, EvaluatorRuleLTL("F goal_reached", -100.f, RewardPriority::GOAL)});
    automata[0].insert({Rule::NO_COLLISION, EvaluatorRuleLTL("G !collision", -1.0f, RewardPriority::SAFETY)});
    automata[0].insert({Rule::LEAVE_INTERSECTION,
                        EvaluatorRuleLTL("G(at_hp_xing -> X !at_hp_xing)", -300.f, RewardPriority::SAFETY)});
    automata[0].insert({Rule::GIVE_WAY,
                        EvaluatorRuleLTL("G(other_near -> !at_hp_xing)", -1.0f, RewardPriority::LEGAL_RULE)});

    for (size_t i = 1; i < automata.size(); ++i) {
      automata[i] = automata[0];
    }

    create_state();
    rewards = std::vector<Reward>(state->get_agent_idx().size(), Reward::Zero());
    jt = JointAction(2, (int) Actions::FORWARD);
  }
  CrossingTestEnv() : CrossingTestEnv(make_default_mcts_parameters(), make_default_crossing_state_parameters()) {};
  ~CrossingTestEnv() {
    LOG(INFO) << "Ego positions:" << pos_history;
    LOG(INFO) << "Otr positions:" << pos_history_other;
  }

  void create_state() {
    Automata aut_v(automata.size());
    for (size_t i = 0; i < automata.size(); ++i) {
      LOG(INFO) << "Rules for agent " << i << ":";
      auto it = automata[i].begin();
      for (size_t j = 0; j < automata[i].size(); ++j) {
        LOG(INFO) << it->second;
        aut_v[i].emplace_back(it->second);
        ++it;
      }
    }
    state = std::make_shared<CrossingState>(aut_v, label_evaluators, crossing_state_parameter_);
  }

  const JointAction &get_jt() const {
    return jt;
  }
  void set_jt(const JointAction &jt) {
    CrossingTestEnv::jt = jt;
    action_history.emplace_back(jt);
  }
  const std::deque<JointAction> &get_action_history() const {
    return action_history;
  }
  void set_automata(const std::vector<std::map<Rule, EvaluatorRuleLTL>> &automata) {
    CrossingTestEnv::automata = automata;
    create_state();
  }
  std::vector<std::map<Rule, EvaluatorRuleLTL>> get_automata() const {
    return automata;
  }
  MctsParameters const mcts_parameters_;
  CrossingStateParameter const crossing_state_parameter_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
  std::vector<Reward> rewards;
  std::vector<std::size_t> pos_history;
  std::vector<size_t> pos_history_other;
  Mcts<CrossingState, Stats, Stats, Heuristic> mcts;
  std::shared_ptr<CrossingState> state;
 protected:
  std::vector<std::map<Rule, EvaluatorRuleLTL>> automata;

 private:
  JointAction jt;
  std::deque<JointAction> action_history;
};

#endif  // MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
