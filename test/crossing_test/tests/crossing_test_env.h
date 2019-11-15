//
// Created by Luis Gressenbuch on 04.11.19.
// Copyright (c) 2019 Luis Gressenbuch. All rights reserved.
//

#ifndef MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
#define MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_

#include <utility>

#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"
#include "mcts/heuristics/random_heuristic.h"
#include "test/crossing_test/crossing_state.hpp"
#include "test/crossing_test/viewer.h"
#include "mcts/random_generator.h"
#include "test/crossing_test/label_evaluator/evaluator_label_collision.h"
#include "test/crossing_test/label_evaluator/evaluator_label_goal_reached.h"
#include "test/crossing_test/label_evaluator/evaluator_label_hold_at_xing.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_near.h"
#include "test/crossing_test/label_evaluator/evaluator_label_speed.h"
#include "test/crossing_test/label_evaluator/evaluator_label_other_goal_reached.h"
#include "test/crossing_test/common.hpp"
#include "mcts/statistics/pareto_uct_statistic.h"
#include "mcts/statistics/slack_uct_statistic.h"
#include "common.h"

using namespace mcts;

enum Rule {
  NO_COLLISION = 0, REACH_GOAL, NO_SPEEDING, GIVE_WAY, LEAVE_INTERSECTION, REACH_GOAL_FIRST, NUM,
};

class BaseTestEnv {
 public:
  BaseTestEnv(MctsParameters mcts_parameters = make_default_mcts_parameters(),
              CrossingStateParameter crossing_state_parameter = make_default_crossing_state_parameters(),
              std::vector<std::map<Rule, EvaluatorRuleLTL>> automata = make_default_automata(
                  make_default_crossing_state_parameters().num_other_agents + 1),
              std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators = make_default_labels(
                  make_default_crossing_state_parameters()))
      : mcts_parameters_(std::move(mcts_parameters)),
        crossing_state_parameter_(std::move(crossing_state_parameter)),
        label_evaluators_(std::move(label_evaluators)),
        rewards(crossing_state_parameter.num_other_agents + 1, Reward::Zero()),
        automata_(std::move(automata)),
        jt(2, static_cast<int>(Actions::FORWARD)) {
    create_state();
  }
  ~BaseTestEnv() {
    LOG(INFO) << "Ego positions:" << pos_history;
    LOG(INFO) << "Otr positions:" << pos_history_other;
  }

  static std::vector<std::map<Rule, EvaluatorRuleLTL>> make_default_automata(size_t num_agents) {
    std::vector<std::map<Rule, EvaluatorRuleLTL>> automata(num_agents);
    automata[0].insert({Rule::NO_SPEEDING, EvaluatorRuleLTL("G !speeding", -1.0f, RewardPriority::LEGAL_RULE_B)});
    automata[0].insert({Rule::REACH_GOAL, EvaluatorRuleLTL("F goal_reached", -100.f, RewardPriority::GOAL)});
    automata[0].insert({Rule::NO_COLLISION, EvaluatorRuleLTL("G !collision", -1.0f, RewardPriority::SAFETY)});
    automata[0].insert({Rule::LEAVE_INTERSECTION,
                        EvaluatorRuleLTL("G(at_hp_xing -> X !at_hp_xing)", -300.f, RewardPriority::SAFETY)});
    automata[0].insert({Rule::GIVE_WAY, EvaluatorRuleLTL("G(other_near -> !at_hp_xing)", -1.0f, RewardPriority::LEGAL_RULE)});

    for (size_t i = 1; i < automata.size(); ++i) {
      automata[i] = automata[0];
    }
    return automata;
  }

  static std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> make_default_labels(CrossingStateParameter params) {
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> labels;
    labels.emplace_back(std::make_shared<EvaluatorLabelCollision>("collision", params.crossing_point));
    labels.emplace_back(std::make_shared<EvaluatorLabelGoalReached>("goal_reached", params.ego_goal_reached_position));
    labels.emplace_back(std::make_shared<EvaluatorLabelHoldAtXing>("at_hp_xing", params.crossing_point + 1));
    labels.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
    labels.emplace_back(std::make_shared<EvaluatorLabelSpeed>("speeding"));
    labels.emplace_back(std::make_shared<EvaluatorLabelOtherGoalReached>("other_goal_reached",
                                                                         params.ego_goal_reached_position));
    return labels;
  }

  virtual JointAction search(size_t num_iterations) = 0;

  const JointAction &get_jt() const {
    return jt;
  }
  void set_jt(const JointAction &jt) {
    BaseTestEnv::jt = jt;
    action_history.emplace_back(jt);
  }
  const std::deque<JointAction> &get_action_history() const {
    return action_history;
  }
  MctsParameters const mcts_parameters_;
  CrossingStateParameter const crossing_state_parameter_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators_;
  std::vector<Reward> rewards;
  std::vector<std::size_t> pos_history;
  std::vector<size_t> pos_history_other;
  std::shared_ptr<CrossingState> state;
 protected:
  std::vector<std::map<Rule, EvaluatorRuleLTL>> automata_;

 private:
  void create_state() {
    Automata aut_v(automata_.size());
    for (size_t i = 0; i < automata_.size(); ++i) {
      LOG(INFO) << "Rules for agent " << i << ":";
      auto it = automata_[i].begin();
      for (size_t j = 0; j < automata_[i].size(); ++j) {
        LOG(INFO) << it->second;
        aut_v[i].emplace_back(it->second);
        ++it;
      }
    }
    state = std::make_shared<CrossingState>(aut_v, label_evaluators_, crossing_state_parameter_);
  }

  JointAction jt;
  std::deque<JointAction> action_history;
};

template<class Stats = UctStatistic<>, class Heuristic = RandomHeuristic>
class CrossingTestEnv : public BaseTestEnv {
 public:
  CrossingTestEnv(MctsParameters mcts_parameters = make_default_mcts_parameters(),
                  CrossingStateParameter crossing_state_parameter = make_default_crossing_state_parameters(),
                  std::vector<std::map<Rule, EvaluatorRuleLTL>> automata = BaseTestEnv::make_default_automata(
                      make_default_crossing_state_parameters().num_other_agents + 1),
                  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators = BaseTestEnv::make_default_labels(
                      make_default_crossing_state_parameters())) : BaseTestEnv(mcts_parameters,
                                                                               crossing_state_parameter,
                                                                               automata,
                                                                               label_evaluators),
                                                                   mcts(mcts_parameters_) {}
  JointAction search(size_t num_iterations) override {
    mcts.search(*(this->state), 50000, num_iterations);
    this->set_jt(mcts.returnBestAction());
    return this->get_jt();
  }
  Mcts<CrossingState, Stats, Stats, Heuristic> mcts;
};

#endif  // MAMCTS_TEST_CROSSING_TEST_CROSSING_TEST_ENV_H_
