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
    added = false; 
    subtracted = false;
    param_index = 0;
}

void PID::UpdateError(double cte, double tolerance) {
    unsigned int LOWER = 100, UPPER = 3500;

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    TotalError( cte );
    step++;

    if( step >= LOWER &&
        step <= UPPER &&
        step % 80 == 0 ){
        
        if( step == LOWER ){
            best_err = total_err;
            return;
        }

        cout << "Iteration: " << step << endl;
        Twiddle( tolerance );
    }
}

double PID::TotalError( double cte) {
    total_err += ( cte*cte );
    return total_err ;
}

void PID::Twiddle( double tolerance ){

    double params[] = { Kp, Ki, Kd };
    double dp_sum = dp[0] + dp[1] + dp[2] ;

    if( dp_sum > tolerance ){

        if( total_err < best_err ){
            best_err = total_err;
            dp[param_index] *= 1.1;
            added = subtracted = false;
            param_index = ( param_index + 1 ) % 3;
        }

        if( !added && !subtracted ){
            added = true;
            params[ param_index ] += dp[ param_index ];
        }
        else if ( added && !subtracted ) {
            subtracted = true;
            params[ param_index ] -= 2*dp[ param_index ];
        }
        else {
            params[ param_index ] += dp[ param_index ];
            dp[ param_index ] *= 0.9;
            added = subtracted = false;
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
