//
// Created by Luis Gressenbuch on 02.01.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#include "test/crossing_test/tests/base_test_env.h"

#include <utility>

BaseTestEnv::BaseTestEnv(
    MctsParameters mcts_parameters,
    CrossingStateParameter crossing_state_parameter,
    std::vector<std::map<Rule, EvaluatorRuleLTLSPtr>> automata,
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators)
    : mcts_parameters_(std::move(mcts_parameters)),
      crossing_state_parameter_(std::move(crossing_state_parameter)),
      label_evaluators_(std::move(label_evaluators)),
      rewards(crossing_state_parameter.num_other_agents + 1,
              Reward::Zero(mcts_parameters.REWARD_VEC_SIZE)),
      automata_(std::move(automata)),
      jt(2, static_cast<int>(Actions::FORWARD)) {
  create_state();
}
std::vector<std::map<Rule, EvaluatorRuleLTLSPtr>>
BaseTestEnv::make_default_automata(size_t num_agents) {
  std::vector<std::map<Rule, EvaluatorRuleLTLSPtr>> automata(num_agents);
  automata[0].insert({Rule::NO_SPEEDING,
                      EvaluatorRuleLTL::make_rule(
                          "G !speeding", -1.0f, RewardPriority::LEGAL_RULE_B)});
  automata[0].insert(
      {Rule::REACH_GOAL, EvaluatorRuleLTL::make_rule("F goal_reached", -100.f,
                                                     RewardPriority::GOAL)});
  automata[0].insert(
      {Rule::NO_COLLISION, EvaluatorRuleLTL::make_rule(
                               "G !collision", -1.0f, RewardPriority::SAFETY)});
  automata[0].insert(
      {Rule::LEAVE_INTERSECTION,
       EvaluatorRuleLTL::make_rule("G(at_xing -> X !at_xing)", -1.0f,
                                   RewardPriority::SAFETY)});
  automata[0].insert({Rule::GIVE_WAY, EvaluatorRuleLTL::make_rule(
                                          "G(other_near -> !at_xing)", -1.0f,
                                          RewardPriority::LEGAL_RULE)});

  for (size_t i = 1; i < automata.size(); ++i) {
    automata[i] = automata[0];
  }
  return automata;
}
std::vector<std::shared_ptr<EvaluatorLabelBase<World>>>
BaseTestEnv::make_default_labels(CrossingStateParameter params) {
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> labels;
  labels.emplace_back(std::make_shared<EvaluatorLabelCollision>(
      "collision", params.crossing_point));
  labels.emplace_back(std::make_shared<EvaluatorLabelGoalReached>(
      "goal_reached", params.ego_goal_reached_position));
  labels.emplace_back(std::make_shared<EvaluatorLabelAtPosition>(
      "at_xing", params.crossing_point));
  labels.emplace_back(std::make_shared<EvaluatorLabelOtherNear>("other_near"));
  labels.emplace_back(std::make_shared<EvaluatorLabelSpeed>("speeding"));
  labels.emplace_back(std::make_shared<EvaluatorLabelOtherGoalReached>(
      "other_goal_reached", params.ego_goal_reached_position));
  return labels;
}
RuleStateMap BaseTestEnv::get_automata_vec() const {
  RuleStateMap aut_v(automata_.size());
  for (size_t i = 0; i < automata_.size(); ++i) {
    LOG(INFO) << "Rules for agent " << i << ":";
    for (const auto &it : automata_[i]) {
      LOG(INFO) << *(it.second);
      auto rule_states = it.second->make_rule_state();
      for (const auto &rs : rule_states) {
        aut_v[i].insert({it.first, rs});
      }
    }
  }
  return aut_v;
}
void BaseTestEnv::create_state() {
  RuleStateMap aut_v = get_automata_vec();
  state = std::make_shared<CrossingState>(aut_v, label_evaluators_,
                                          crossing_state_parameter_);
}
void BaseTestEnv::set_jt(const JointAction &jt) {
  BaseTestEnv::jt = jt;
  action_history.emplace_back(jt);
}
const std::deque<JointAction> &BaseTestEnv::get_action_history() const {
  return action_history;
}
const JointAction &BaseTestEnv::get_jt() const { return jt; }
BaseTestEnv::~BaseTestEnv() {
  LOG(INFO) << "Ego positions:" << pos_history;
  LOG(INFO) << "Otr positions:" << pos_history_other;
}
