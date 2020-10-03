//
// Created by Luis Gressenbuch on 01.04.20.
// Copyright (c) 2020 Luis Gressenbuch. All rights reserved.
//

#ifndef MVMCTS_STATISTICS_THRESHOLD_COMPERATOR_H_
#define MVMCTS_STATISTICS_THRESHOLD_COMPERATOR_H_

namespace mvmcts {
template <class T>
struct ThresholdComparator {
  explicit ThresholdComparator(const T &threshold) : threshold_(threshold) {}
  const T threshold_;
  bool operator()(const T &lhs, const T &rhs) const {
    assert(lhs.rows() == rhs.rows() && lhs.rows() == threshold_.rows());
    for (auto lhs_iter = lhs.begin(), rhs_iter = rhs.begin(),
              thres_iter = threshold_.begin();
         lhs_iter != lhs.end(); ++lhs_iter, ++rhs_iter, ++thres_iter) {
      if ((*lhs_iter > *thres_iter && *rhs_iter > *thres_iter) ||
          (*lhs_iter == *rhs_iter)) {
        continue;
      } else {
        return *lhs_iter < *rhs_iter;
      }
    }
    return false;
  }
};
}  // namespace mvmcts

#endif  //  MVMCTS_STATISTICS_THRESHOLD_COMPERATOR_H_
