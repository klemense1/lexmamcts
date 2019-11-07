// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef CROSSINGSTATE_HPP
#define CROSSINGSTATE_HPP

#include <map>
#include <iostream>
#include <random>

#include "evaluator_rule_ltl.hpp"
#include "test/crossing_test/common.hpp"
#include "test/crossing_test/evaluator_label_base.hpp"

using namespace mcts;
using namespace modules::models::behavior;

namespace mcts {
class Viewer;
}

typedef std::vector<std::vector<EvaluatorRuleLTL>> Automata;

// A simple environment with a 1D state, only if both agents select different actions, they get nearer to the terminal state
class CrossingState : public mcts::StateInterface<CrossingState> {

 public:
  static const unsigned int num_other_agents = 1;
  static const int state_x_length = 13;
  static const int ego_goal_reached_position = 13;
  static const int terminal_depth_ = 30;
  static constexpr float ALPHA = 10.0f;
  static constexpr int crossing_point = (state_x_length - 1) / 2 + 1;

    CrossingState(Automata &automata,
                  const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator) :
        agent_states_(num_other_agents + 1),
        terminal_(false),
        automata_(automata),
        label_evaluator_(label_evaluator),
        depth_(0) {
        for (auto &state : agent_states_) {
            state = AgentState();
        }
    }

    CrossingState(const std::vector<AgentState> &agent_states,
                  const bool terminal,
                  Automata &automata,
                  const std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> &label_evaluator, int depth = 0) :
        agent_states_(agent_states),
        terminal_(terminal),
        automata_(automata),
        label_evaluator_(label_evaluator),
        depth_(depth){};
    ~CrossingState() {};

    std::shared_ptr<CrossingState> clone() const {
        return std::make_shared<CrossingState>(*this);
    }

    template<typename ActionType = Actions>
    ActionType get_last_action(const AgentIdx &agent_idx) const {
        return agent_states_[agent_idx].last_action;
    }

    std::shared_ptr<CrossingState> execute(const JointAction &joint_action, std::vector<Reward> &rewards) {

        EvaluationMap labels;
        Automata next_automata(automata_);
        World next_world;
        bool terminal;
        std::vector<AgentState> next_agent_states(agent_states_.size());
        rewards.resize(num_other_agents + 1);

        // CALCULATE NEXT STATE
        for (size_t i = 0; i < agent_states_.size(); ++i) {
            const auto &old_state = agent_states_[i];
            int new_x = old_state.x_pos + static_cast<int>(aconv(joint_action[i]));
            next_agent_states[i] = AgentState(new_x, aconv(joint_action[i]));
        }
        labels["ego_out_of_map"] = false;
        if (next_agent_states[ego_agent_idx].x_pos < 0) {
            labels["ego_out_of_map"] = true;
        }

        // REWARD GENERATION
        // For each agent
        for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
            // Labeling
            std::vector<AgentState> next_other_agents(next_agent_states);
            // TODO: Improve efficiency
            next_other_agents.erase(next_other_agents.begin() + agent_idx);
            // Create perspective from current agent
            next_world = World(next_agent_states[agent_idx], next_other_agents);

            for (auto le : label_evaluator_) {
                labels[le->get_label_str()] = le->evaluate(next_world);
            }
            assert(ego_agent_idx == 0);
            if (agent_idx == ego_agent_idx) {
                terminal = labels["goal_reached"] || labels["collision"] || (depth_ + 1 >= terminal_depth_);
            }
            rewards[agent_idx] = Reward::Zero();

            // Automata transit
            for (EvaluatorRuleLTL &aut : (next_automata[agent_idx])) {
                rewards[agent_idx](aut.get_type()) += aut.evaluate(labels);
            }
            rewards[agent_idx] += get_action_cost(joint_action[agent_idx]);
            labels.clear();
        } // End for each agent

        return std::make_shared<CrossingState>(next_agent_states,
                                               terminal,
                                               next_automata,
                                               label_evaluator_, depth_ + 1);
    }

    std::vector<Reward> get_final_reward() const {
        std::vector<Reward> rewards(agent_states_.size(), Reward::Zero());
        for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
            // Automata transit
            for (EvaluatorRuleLTL const &aut : (automata_[agent_idx])) {
                rewards[agent_idx](aut.get_type()) += aut.get_final_reward();
            }
            // Reward for goal proximity
            //rewards[agent_idx](RewardPriority::GOAL) += ALPHA * fmin(agent_states_[agent_idx].x_pos, ego_goal_reached_position);
        }
        return rewards;
    }

    ActionIdx get_num_actions(AgentIdx agent_idx) const {
        return static_cast<size_t>(Actions::NUM);
    }

    bool is_terminal() const {
        return terminal_;
    }

    const std::vector<AgentIdx> get_agent_idx() const {
        std::vector<AgentIdx> agent_idx(num_other_agents + 1);
        std::iota(agent_idx.begin(), agent_idx.end(), 0);
        return agent_idx; // adapt to number of agents
    }

    std::string sprintf() const {
        std::stringstream ss;
        ss << "Ego: x=" << agent_states_[ego_agent_idx].x_pos;
        for (size_t i = 1; i < agent_states_.size(); ++i) {
            ss << ", Ag" << i << ": x=" << agent_states_[i].x_pos;
        }
        ss << std::endl;
        return ss.str();
    }

    bool ego_goal_reached() const {
        return agent_states_[ego_agent_idx].x_pos >= ego_goal_reached_position;
    }

    inline int distance_to_ego(const AgentIdx &other_agent_idx) const {
        return agent_states_[ego_agent_idx].x_pos - agent_states_[other_agent_idx + 1].x_pos;
    }

    typedef Actions ActionType;

    int get_ego_pos() const {
        return agent_states_[ego_agent_idx].x_pos;
    }

    const std::vector<AgentState> &get_agent_states() const {
        return agent_states_;
    }

    void draw(Viewer *viewer) const;

    void reset_depth() {
      depth_ = 0;
    }

  void update_rule_belief();

  void reset_violations();

 private:
    Reward get_action_cost(ActionIdx action);

    std::vector<AgentState> agent_states_;
    bool terminal_;
    Automata automata_;
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator_;
    int depth_;
};

#endif
