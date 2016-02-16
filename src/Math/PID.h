#pragma once

#include <Base.h>
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
	float Kp, Ki, Kd;
	float I_out_total;
	float Input_prev, Input_prev2;
	float Output, Input;
	const float CLIP_MIN = -1.0f;
	const float CLIP_MAX = 1.0f;

	eIntegrateType IntType;
	
	void Clip();
};