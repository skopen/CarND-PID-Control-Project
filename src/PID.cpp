#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	//std::cout << "Initing PID: " << Kp << ", " << Ki << ", " << Kd << std::endl;

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = 0;
	i_error = 0;
	d_error = 0;
	prevCte = 0;
	control = 0;
	totalError = 0;
}

void PID::UpdateError(double cte) {

	p_error = cte;
	d_error = cte - prevCte;
	i_error += cte;

	prevCte = cte;

	control = -Kp*p_error - Kd*d_error - Ki*i_error;

	if (control > 1.0)
		control = 1.0;

	if (control < -1.0)
		control = -1.0;

	totalError += cte*cte;
}

double PID::TotalError()
{
	return totalError;
}

void PID::ResetTotalError()
{
	totalError = 0;
}

double PID::getControl() {

	return control;
}
