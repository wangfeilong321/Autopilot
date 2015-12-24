#ifndef PID_H
#define PID_H

#include <Interface.h>

class PID : public Interface {
public:
	PID();
	virtual ~PID() = default;
	
	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();
	
	/// These define the indices use to select the various integrators.
	enum eIntegrateType { eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3 };
	
	void SetCurrentInput(double angleRad);
	
	double GetCurrentOutput();

private:
	double Kp, Ki, Kd;
	double I_out_total;
	double Input_prev, Input_prev2;
	double KpPropertySign;
	double KiPropertySign;
	double KdPropertySign;
	double Output, Input;
	double clipmin, clipmax;
	bool IsStandard;
	double dt;
	double Trigger;

	eIntegrateType IntType;
	
	void Clip();
};

#endif