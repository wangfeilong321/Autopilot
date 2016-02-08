#ifndef PID_H
#define PID_H

#include <Interface.h>
#include <StateSpace.h>

class PID : public Interface {
public:
	PID();
	virtual ~PID() = default;
	
	virtual void Connect();
	virtual bool Connected();
	virtual bool Run();

	void SetInput(float val);
	float GetOutput();

private:
	enum eIntegrateType { eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3 };

	float Kp, Ki, Kd;
	float I_out_total;
	float Input_prev, Input_prev2;
	float Output, Input;
	const float CLIP_MIN = -1.0f;
	const float CLIP_MAX = 1.0f;
	float dt;

	eIntegrateType IntType = eAdamsBashforth2;
	
	void Clip();
};

#endif