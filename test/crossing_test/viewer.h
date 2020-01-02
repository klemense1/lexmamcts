// Copyright (c) 2019 Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef TEST_CROSSING_TEST_VIEWER_H_
#define TEST_CROSSING_TEST_VIEWER_H_

#include <tuple>
#include <utility>

namespace mcts {

typedef std::tuple<float, float, float, float> Color;  // < R;G;B;ALPHA

class Viewer {
 public:
  Viewer() {}
  virtual ~Viewer() {}
  virtual void drawPoint(float x, float y, float size, Color color) = 0;
  virtual void drawLine(std::pair<float, float> x, std::pair<float, float> y,
                        float linewidth, Color color) = 0;
};

}  // namespace mcts

#endif  // TEST_CROSSING_TEST_VIEWER_H_
