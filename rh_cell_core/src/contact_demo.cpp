// Contact demo: cassette moves at 10 mm/s towards a stiff wall, commanded 10 mm INTO it.
// Run: ros2 run rh_cell_core contact_demo > contact.csv   (summary printed on screen)
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rh_cell_core/admittance_1d.hpp"
#include "rh_cell_core/force_guard.hpp"

int main()
{
  const double dt = 0.002, wall = 0.0, k_env = 50000.0;
  const double start = -0.05, target = 0.01, speed = 0.01, force_limit = 20.0;

  rh_cell_core::Admittance1D ctrl({2.0, 60.0, 500.0}, start);
  rh_cell_core::ForceGuard guard(force_limit, 5);

  double peak = 0.0, f = 0.0;
  std::printf("t,x_des,x,force,tripped\n");
  for (int i = 0; i <= static_cast<int>(8.0 / dt); ++i)
  {
    const double t = i * dt;
    const double x_des = std::min(target, start + speed * t);
    const double x = ctrl.step(x_des, f, dt);
    f = (x > wall) ? -k_env * (x - wall) : 0.0;
    peak = std::max(peak, std::abs(f));
    const bool tripped = guard.update(f);
    std::printf("%.3f,%.6f,%.6f,%.3f,%d\n", t, x_des, x, f, tripped ? 1 : 0);
  }
  std::fprintf(stderr, "\n=== Contact demo summary ===\n");
  std::fprintf(stderr, "Admittance control: peak force %.1f N, final force %.1f N, guard %s\n", peak,
               std::abs(f), guard.tripped() ? "TRIPPED" : "ok");
  std::fprintf(stderr, "Pure position control would push with about %.0f N (k_env x 10 mm).\n",
               k_env * (target - wall));
  return 0;
}
