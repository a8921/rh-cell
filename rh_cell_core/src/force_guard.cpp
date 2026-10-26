#include "rh_cell_core/force_guard.hpp"

#include <cmath>

namespace rh_cell_core
{
ForceGuard::ForceGuard(double limit, int samples_to_trip) : limit_(limit), samples_to_trip_(samples_to_trip) {}

bool ForceGuard::update(double force)
{
  if (tripped_)
    return true;  // latched: stays tripped until reset()

  if (std::abs(force) > limit_)
    count_ = count_ + 1;
  else
    count_ = 0;

  if (count_ >= samples_to_trip_)
    tripped_ = true;
  return tripped_;
}

void ForceGuard::reset()
{
  tripped_ = false;
  count_ = 0;
}
}  // namespace rh_cell_core
