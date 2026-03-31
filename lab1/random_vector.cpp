#include "random_vector.h"
// TODO: add any include you might require

RandomVector::RandomVector(int size, double max_val) { 
  vect.resize(size);
  const double scale = max_val / (static_cast<double>(RAND_MAX) + 1.0);
  for (int i = 0; i < size; ++i) {
    vect[i] = std::rand() * scale;
  }
}

void RandomVector::print(){
  for (std::size_t i = 0; i < vect.size(); ++i) {
    if (i != 0) {
      std::cout << ' ';
    }
    std::cout << vect[i];
  }
  std::cout << '\n';
}

double RandomVector::mean(){
  if (vect.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (double value : vect) {
    sum += value;
  }

  return sum / static_cast<double>(vect.size());
}

double RandomVector::max(){
  if (vect.empty()) {
    return 0.0;
  }

  double max_value = vect[0];
  for (std::size_t i = 1; i < vect.size(); ++i) {
    if (vect[i] > max_value) {
      max_value = vect[i];
    }
  }

  return max_value;
}

double RandomVector::min(){
  if (vect.empty()) {
    return 0.0;
  }

  double min_value = vect[0];
  for (std::size_t i = 1; i < vect.size(); ++i) {
    if (vect[i] < min_value) {
      min_value = vect[i];
    }
  }

  return min_value;
}

void RandomVector::printHistogram(int bins){
  if (bins <= 0 || vect.empty()) {
    return;
  }

  double min_value = vect[0];
  double max_value = vect[0];
  for (std::size_t i = 1; i < vect.size(); ++i) {
    const double value = vect[i];
    if (value < min_value) {
      min_value = value;
    }
    if (value > max_value) {
      max_value = value;
    }
  }

  std::vector<int> counts(static_cast<std::size_t>(bins), 0);

  if (max_value == min_value) {
    counts[0] = static_cast<int>(vect.size());
  } else {
    const double scale = static_cast<double>(bins) / (max_value - min_value);
    for (double value : vect) {
      int idx = static_cast<int>((value - min_value) * scale);
      if (idx >= bins) {
        idx = bins - 1;
      }
      ++counts[static_cast<std::size_t>(idx)];
    }
  }

  int max_count = counts[0];
  for (int i = 1; i < bins; ++i) {
    if (counts[static_cast<std::size_t>(i)] > max_count) {
      max_count = counts[static_cast<std::size_t>(i)];
    }
  }

  for (int level = max_count; level > 0; --level) {
    int last_filled = -1;
    for (int i = 0; i < bins; ++i) {
      if (counts[static_cast<std::size_t>(i)] >= level) {
        last_filled = i;
      }
    }

    for (int i = 0; i <= last_filled; ++i) {
      if (i != 0) {
        std::cout << ' ';
      }
      std::cout << (counts[static_cast<std::size_t>(i)] >= level ? "***" : "   ");
    }
    std::cout << '\n';
  }
}
