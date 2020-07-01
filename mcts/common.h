// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef MCTS_COMMON_H
#define MCTS_COMMON_H


namespace mcts {

class MctsTest;

#ifdef UNIT_TESTING
#define MCTS_TEST friend class MctsTest;
#else
#define MCTS_TEST
#endif


#ifdef CRTP_DYNAMIC_INTERFACE
#define CRTP_INTERFACE(type) inline Implementation& impl()  { \
        auto derivedptr = dynamic_cast<type*>(this); \
        MCTS_EXPECT_TRUE(derivedptr!=nullptr); \
        return *derivedptr; \
        }
#else
#define CRTP_INTERFACE(type) inline Implementation& impl() {\
        return *static_cast<type*>(this); \
        }
#endif

#ifdef CRTP_DYNAMIC_INTERFACE
#define CRTP_CONST_INTERFACE(type) inline const Implementation& impl() const { \
        auto const derivedptr = dynamic_cast<const type*>(this); \
        MCTS_EXPECT_TRUE(derivedptr!=nullptr); \
        return *derivedptr; \
    }
#else
#define CRTP_CONST_INTERFACE(type) inline const Implementation& impl() const { \
        return *static_cast<const type*>(this); \
        }
#endif

#define MCTS_EXPECT_TRUE(cond)

  template<typename T>
  inline std::ostream& operator<<(std::ostream& os, std::vector<T> const& d) {
    os << "[";
    for(typename std::vector<T>::const_iterator ii = d.begin(); ii != d.end(); ++ii) {
      os << *ii;
      if(ii >= d.end() - 1) {
        break;
      }
      os << ", ";
    }
    os << "]";
    return os;
  }

} // namespace mcts
#endif
