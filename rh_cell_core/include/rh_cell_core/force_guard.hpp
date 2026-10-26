#pragma once
// Force guard: a tiny software interlock.
// Trips when |force| stays above `limit` for `samples_to_trip` consecutive samples
// (one spike must NOT trip it). Latched until reset(), like an industrial interlock.

namespace rh_cell_core
{
class ForceGuard
{
public:
  ForceGuard(double limit, int samples_to_trip);
  bool update(double force);  // returns true if tripped
  bool tripped() const { return tripped_; }
  void reset();

private:
  double limit_;
  int samples_to_trip_;
  int count_ = 0;
  bool tripped_ = false;
};
}  // namespace rh_cell_core
