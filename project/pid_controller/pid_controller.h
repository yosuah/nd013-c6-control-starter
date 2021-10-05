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
  	double cte;
  	double diff_cte;
  	double sum_cte;

    /*
    * Coefficients
    */
  	double Kp;
  	double Ki;
  	double Kd;

    /*
    * Output limits
    */
  	double output_lim_max;
  	double output_lim_min;
  
    /*
    * Delta time
    */
  	double delta_time;

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
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError(bool debug=false);
  
    /*
    * Update the delta time.
    */
    void UpdateDeltaTime(double new_delta_time);
  
protected:
	/*
    * Store if this is the first iteration, as diff_cte is set to 0 in that case.
    */
	bool is_first;  
};

#endif //PID_CONTROLLER_H


