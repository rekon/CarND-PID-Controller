#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0;
    i_error = 0;
    d_error = 0;
    step = 0;
    dp = new double[3]{ 0.1*Kp, 0.1*Ki, 0.1*Kd };
    total_err = 0;
    best_err = 1e9+7;
    should_add = false; 
    should_subtract = false;
    param_index = 0;
}

void PID::UpdateError(double cte, double tolerance) {
    double params[] = { Kp, Ki, Kd };
    double dp_sum = dp[0] + dp[1] + dp[2] ;
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    TotalError( cte );
    step++;

    if( step > 100 && step % 20 == 0 && dp_sum > 1e-4 ){
        cout << "Iteration: " << step << endl;
        cout << "Current total_error: " << total_err << endl;

        if( step == 100 ){
            best_err = total_err;
            return;
        }

        if( total_err < best_err ){
            best_err = total_err;
            dp[param_index] *= 1.1;
            should_add = should_subtract = false;
            param_index = ( param_index + 1 ) % 3;
        }

        if( !should_add && !should_subtract){
            params[ param_index ] += dp[ param_index ];
            should_add = true;
        }
        else if ( should_add && !should_subtract ) {
            params[ param_index ] -= 2*dp[ param_index ];
            should_subtract = true;
        }
        else {
            params[ param_index ] += dp[ param_index ];
            dp[ param_index ] *= 0.9;
            should_add = should_subtract = false;
            param_index = ( param_index + 1 ) % 3;         
        }

        total_err = 0;

        Kp = params[0];
        Ki = params[1];
        Kd = params[2];

        cout<<"\nNew Params :\n";
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
    }
}

double PID::TotalError( double cte) {
    total_err += ( cte*cte );
    return total_err ;
}
