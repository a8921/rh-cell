#pragma once
// 1-D admittance controller along the insertion axis.
//
//     M * a  +  D * v  +  K * (x - x_des)  =  f_ext
//
// f_ext = force the environment applies on the tool along +x (wall pushing back = NEGATIVE).
// Free space: f_ext = 0, the tool follows x_des. On contact the virtual spring K yields,
// so the robot pushes with about K * (how far x_des is inside the wall).
// No ROS here on purpose: usable in ROS 2, CERNTAURO, or a unit test.

namespace rh_cell_core
{
struct AdmittanceParams
{
  double mass = 2.0;         // M  [kg]
  double damping = 60.0;     // D  [N*s/m]
  double stiffness = 500.0;  // K  [N/m]
};

class Admittance1D
{
public:
  explicit Admittance1D(const AdmittanceParams& p, double x0 = 0.0);
  // One control period. Returns the commanded position to send to the robot.
  double step(double x_des, double f_ext, double dt);
  double position() const { return x_; }
  double velocity() const { return v_; }

private:
  AdmittanceParams p_;
  double x_;
  double v_ = 0.0;
};
}  // namespace rh_cell_core
