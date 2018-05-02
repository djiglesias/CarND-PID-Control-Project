#include "PID.h"
#include <iostream>

using namespace std;

PID::PID(): is_initialized(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	// Set controller gains.
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	// Initialize controller errors.
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

}

void PID::SetGains(double *gains) {
	this->Kp = gains[0];
	this->Kd = gains[1];
	this->Ki = gains[2];
}

void PID::UpdateError(double cte) {

	// Check if first reading.
	if (!is_initialized) {
		p_error = cte;
		is_initialized = true;
	}

	// Set previous error state.
	double prev_cte = p_error;

	// Update errors.
	p_error = cte;
	i_error += cte;
	d_error = p_error - prev_cte;

}

double PID::TotalError() {
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}

