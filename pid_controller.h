#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
 public:
  PIDController(double kp, double ki, double kd, double min_output,
                double max_output);

  void UpdateError(double error, double dt);
  double TotalError() const;
  void Reset();

 private:
  double kp_;
  double ki_;
  double kd_;
  double min_output_;
  double max_output_;
  double p_error_;
  double i_error_;
  double d_error_;
  double previous_error_;
  bool has_previous_error_;
};

#endif  // PID_CONTROLLER_H
