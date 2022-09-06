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
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  _kp = Kpi;
  _kd = Kdi;
  _ki = Kii;
  _cte = 0.0;
  _prev_cte = 0.0;
  _sum_cte = 0.0;
  _output_lim_max = output_lim_maxi;
  _output_lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {
  _prev_cte = _cte;
  _cte = cte;
  _sum_cte += cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double pid_output = (-_kp * _cte) - (_kd * (_cte - _prev_cte) / _delta_time) - (_ki * _sum_cte * _delta_time);
  	
  	double control =  std::max(std::min(_output_lim_max, pid_output), _output_lim_min);
  
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  _delta_time = new_delta_time;
}