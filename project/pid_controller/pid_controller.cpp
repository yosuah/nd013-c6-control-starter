/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  sum_cte = 0.0;
  diff_cte = 0.0;
  cte = 0.0;
  is_first = true;
}


void PID::UpdateError(double cte_current) {
  if (is_first) {
    	diff_cte = 0;
    	is_first = false;
  } else {
      if (std::abs(delta_time) < 0.0001)
        diff_cte = 0;
      else
		diff_cte = (cte_current - cte) / delta_time * 0.7 + 0.3 * diff_cte;
  }
  sum_cte = sum_cte + cte_current * delta_time;
  cte = cte_current;
}

double PID::TotalError(bool debug) {
  double control;
  
  // Calculate control output
  control = -Kp * cte - Kd * diff_cte - Ki * sum_cte; // NOTE: error terms already include the time
  
  if (debug) {
  	std::cout << "Cte " << cte << " Control: " << control << " P: " << -Kp * cte << " I: " << - Ki * sum_cte << " D: " << - Kd * diff_cte << " T: " << delta_time << std::endl;
  }
  
  // Ensure output is in the desired range
  control = max(control, output_lim_min);
  control = min(control, output_lim_max);
  
  return control;
}

void PID::UpdateDeltaTime(double new_delta_time) {
   delta_time = new_delta_time;
}