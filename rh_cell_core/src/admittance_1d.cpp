#include "rh_cell_core/admittance_1d.hpp"

namespace rh_cell_core
{
Admittance1D::Admittance1D(const AdmittanceParams& p, double x0) : p_(p), x_(x0) {}

double Admittance1D::step(double x_des, double f_ext, double dt)
{
  // TODO 1: solve   M*a + D*v + K*(x - x_des) = f_ext   for the acceleration a.
  //         Available: p_.mass (M), p_.damping (D), p_.stiffness (K), v_ (v), x_ (x), x_des, f_ext.
  //         Move the D and K terms to the right-hand side, then divide by M.
 const double a = (f_ext - p_.damping * v_ - p_.stiffness * (x_ - x_des)) / p_.mass;

  // Semi-implicit Euler integration: update velocity first, then position.
  v_ += a * dt;
  x_ += v_ * dt;
  return x_;
}
}  // namespace rh_cell_core
