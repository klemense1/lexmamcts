// Copyright (c) 2019 Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MVMCTS_STATISTICS_LEXICOGRAPHICAL_COMPERATOR_H_
#define MVMCTS_STATISTICS_LEXICOGRAPHICAL_COMPERATOR_H_

#include <algorithm>
#include <cmath>

namespace mvmcts {
struct LexicographicalComperator {
  explicit LexicographicalComperator(const double eps = 1e-5) : eps_(eps) {}
  template <class T>
  bool operator()(const T &a, const T &b) const {
    return std::lexicographical_compare(
        a.begin(), a.end(), b.begin(), b.end(),
        [this](const typename T::Scalar &a, const typename T::Scalar &b) {
          return (b - a) >
                 ((std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) *
                  eps_);
        });
  }
  const double eps_;
};
}  // namespace mvmcts

#endif  //  MVMCTS_STATISTICS_LEXICOGRAPHICAL_COMPERATOR_H_
