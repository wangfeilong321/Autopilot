#ifndef PID_H
#define PID_H

class PID {
public:
	PID();
	~PID();
	
	bool Run(void);
	
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
	void Debug(int from);
};

#endif