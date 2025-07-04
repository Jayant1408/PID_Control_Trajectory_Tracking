/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/
    /*
    * Errors
    */

    /*
    * Coefficients
    */

    /*
    * Output limits
    */
  
    /*
    * Delta time
    */
   double lim_max_output;
   double lim_min_output;
   double delta_t;
   double error_p;
   double error_i;
   double error_d;
   double k_p;
   double k_i;
   double k_d;
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
    void Init(double Kp, double Ki, double Kd, double output_lim_maxi, double output_lim_mini);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


