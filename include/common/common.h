// Copyright (c) 2020 NRSL HITsz. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 2020/9/26.

#pragma once

#include <algorithm>
#include <chrono>
#include <random>
#include <set>
#include <vector>

#include <glog/logging.h>

/*Output style*/
#define RESET "\033[0m"
#define GREEN "\033[32m"              ///< Green
#define YELLOW "\033[33m"             ///< Yellow
#define BLUE "\033[34m"               ///< Blue
#define MAGENTA "\033[35m"            ///< Magenta
#define CYAN "\033[36m"               ///< Cyan
#define BOLDBLACK "\033[1m\033[30m"   ///< Bold Black
#define BOLDRED "\033[1m\033[31m"     ///< Bold Red
#define BOLDGREEN "\033[1m\033[32m"   ///< Bold Green
#define BOLDYELLOW "\033[1m\033[33m"  ///< Bold Yellow
#define BOLDBLUE "\033[1m\033[34m"    ///< Bold Blue
#define BOLDMAGENTA "\033[1m\033[35m" ///< Bold Magenta
#define BOLDCYAN "\033[1m\033[36m"    ///< Bold Cyan
#define BOLDWHITE "\033[1m\033[37m"   ///< Bold White

#define COLOR_END "\033[0m"

namespace common {

struct Color {
  Color(float red, float green, float blue) : r(red), g(green), b(blue) {}
  float r;
  float g;
  float b;
};

template <typename Iterator>
void TraverseLogOut(Iterator begin, Iterator end, bool with_index = false) {
  if (!with_index) {
    while (begin != end) {
      LOG(INFO) << *begin << std::endl;
      ++begin;
    }
  } else {
    int index = 0;
    while (begin != end) {
      LOG(INFO) << index << ": " << *begin << std::endl;
      ++begin;
      ++index;
    }
  }
}

template <typename Container>
void TraverseLogOut(const Container &container, bool with_index = false) {
  TraverseLogOut(container.begin(), container.end(), with_index);
}

// Return index.
template <typename T>
int FindClosestValue(const std::vector<T> &data /*MUST be sorted*/,
                     const T &val) {
  int found = 0;
  if (data.front() < data.back()) {
    found = std::upper_bound(data.begin(), data.end(), val) - data.begin();
  } else {
    found = data.rend() - std::upper_bound(data.rbegin(), data.rend(), val);
  }
  if (found == 0) {
    return found;
  }
  if (found == data.size()) {
    return found - 1;
  }
  const int diff_next = std::abs(data[found] - val);
  const int diff_prev = std::abs(val - data[found - 1]);
  return diff_next < diff_prev ? found : found - 1;
}

template <typename T>
std::vector<int>
FindClosestValues(const std::vector<T> &data /*MUST be sorted*/, const T &val,
                  int num_closest) {
  // CHECK_GE(data.size(), num_closest);
  const int cloest_ind = FindClosestValue(data, val);
  const int left = std::max<int>(cloest_ind - num_closest + 1, 0);
  const int right =
      std::min<int>(cloest_ind + num_closest - 1, data.size() - 1);

  // NOTE: we didn't use `map` here, cause using double/float as key is not
  // stable.
  std::vector<std::pair<int /*index*/, T /*value*/>> closest_candidates;
  const int closest_candidates_size = right - left + 1;
  closest_candidates.reserve(closest_candidates_size);
  for (int i = left; i <= right; ++i) {
    closest_candidates.emplace_back(i, data[i]);
  }
  std::sort(closest_candidates.begin(), closest_candidates.end(),
            [](const std::pair<int, T> &lhs, const std::pair<int, T> &rhs) {
              return lhs.second < rhs.second;
            });

  const int closest_indices_size =
      std::min(closest_candidates_size, num_closest);
  std::vector<int> closest_indices(closest_indices_size);
  for (int i = 0; i < closest_indices_size; ++i) {
    closest_indices[i] = closest_candidates[i].first;
  }
  return closest_indices;
}

static std::set<int> RandomDownsamplingArray(int max_size,
                                             double remain_ratio /*[0, 1]*/) {
  CHECK_GT(max_size, 0);
  CHECK_GE(remain_ratio, 0);
  CHECK_LE(remain_ratio, 1);

  std::vector<int> indices(max_size);
  std::iota(indices.begin(), indices.end(), 0);

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(indices.begin(), indices.end(),
               std::default_random_engine(seed));

  std::set<int> downsampled_indices;
  const int end_index = max_size * remain_ratio;
  for (int i = 0; i < end_index; ++i) {
    downsampled_indices.insert(indices[i]);
  }

  return downsampled_indices;
}
} // namespace common
