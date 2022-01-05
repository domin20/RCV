/*
 * PID.h
 *
 *  Created on: 21 wrz 2021
 *      Author: Dominik
 */

#ifndef INC_PID_H_
#define INC_PID_H_


class PID {

private:
	float kp;
	float ki;
	float kd;

	float tau_it;
	float tau_dt;
public:
	PID(void);
	PID(float kp);
	PID(float kp, float ki);
	PID(float kp, float ki, float kd);

	void SetNewTauIT(float tau);
	void SetNewTauDT(float tau);
	void PID_init(float kp, float ki, float kd);
	float PIDCalculate(float e, uint16_t *time_cnt);
	float IntegralTermCalculate(float inupt_value, uint16_t *time_cnt);
	float DerivativeTermCalculate(float input_value, uint16_t *time_cnt);
	~PID(){};
};


#endif /* INC_PID_H_ */
