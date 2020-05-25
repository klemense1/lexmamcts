// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================


#ifndef MCTS_EPISODE_RUNNER_H_
#define MCTS_EPISODE_RUNNER_H_

#include "external/com_github_google_glog/_virtual_includes/glog/glog/logging.h"

#include "test/crossing_test/tests/crossing_test_env.h"

namespace mcts {

class CrossingStateEpisodeRunner : public CrossingTestEnv<UctStatistic<>> {
 public:
  CrossingStateEpisodeRunner(const unsigned int max_steps, Viewer *viewer)
      : viewer_(viewer), current_step_(0), MAX_STEPS(max_steps) {};

  void step() {
    if (this->state->is_terminal() || current_step_ > MAX_STEPS) {
      return;
    }
    std::vector<Reward> rewards(2);

    JointAction jointaction(this->state->get_agent_idx().size());
    this->mcts.Search(*(this->state), 5000, 1000);
    jointaction = this->mcts.ReturnBestAction();
    std::cout << "Step " << current_step_ << ", Action = " << jointaction << ", " << this->state->sprintf()
              << std::endl;
    this->state = this->state->execute(jointaction, rewards);

    current_step_ += 1;

    if (viewer_) {
      this->state->draw(viewer_);
    }
  }

 private:
  Viewer *viewer_;
  unsigned int current_step_;
  const unsigned int MAX_STEPS;
};

}  // namespace mcts

#endif // MCTS_EPISODE_RUNNER_H_