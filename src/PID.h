#ifndef PID_H
#define PID_H

#include <iostream>
#include <numeric>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Tuning Parameters
  */
  unsigned long long step;
  double *dp;
  double total_err;
  double best_err;
  bool added; 
  bool subtracted;
  int param_index;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double tolerance = 1e-4);

  /*
  * Calculate the total PID error.
  */
  double TotalError( double cte );

  /*
  * Twiddle.
  */
  void Twiddle( double tolerance );
};

#endif /* PID_H */
