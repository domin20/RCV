/*
 * PID.cpp
 *
 *  Created on: 21 wrz 2021
 *      Author: Dominik
 */
#include "main.h"
#include "iostream"
#include "math.h"
#include "PID.h"


PID::PID()
{
	kp = 1.0f;
	ki = 0.0f;
	kd = 0.0f;
	tau_it = tau_dt = 1.0f;
}
PID::PID(float kp)
{
	this->kp = kp;
	ki = 0;
	kd = 0;
	tau_it = tau_dt = 1.0f;
}
PID::PID(float kp, float ki)
{
	this->kp = kp;
	this->ki = ki;
	kd = 0;
	tau_it = tau_dt = 1.0f;
}
PID::PID(float kp, float ki, float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	tau_it = tau_dt = 1.0f;
}
void PID::PID_init(float kp, float ki, float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	tau_it = tau_dt = 1.0f;
}
void PID::SetNewTauIT(float tau)
{
	tau_it = tau;
}
void PID::SetNewTauDT(float tau)
{
	tau_dt = tau;
}
float PID::PIDCalculate(float e, uint16_t *time_cnt)
{
	float u;

	// variables for integral term ..
	static float input_value_it = 0.0f;
	static float output_value_it = 0.0f;

	// variables for derivative term..
	static float input_value_dt = 0.0f;
	static float output_value_dt = 0.0f;


	// ******************* INTEGRAL TERM *********************
	input_value_it = e*ki;
	output_value_it = IntegralTermCalculate(input_value_it, time_cnt);


	// ******************* DERIVATIVE TERM *********************
	input_value_dt = e*kd;
	output_value_dt = DerivativeTermCalculate(input_value_dt, time_cnt);

	u = kp*e + output_value_it + output_value_dt;
	return u;
}
float PID::IntegralTermCalculate(float input_value, uint16_t *time_cnt)
{
	static float inv_euler = 0.368f;
	static float output_value = 0.0f;
	static uint16_t time_it = 0;

	static float exponent = 0.0f;
	static uint8_t direction = 1;
	static float U_we = 0.0f;
	static float U_value = 0.0f;

	if(time_it > (*time_cnt)) {
		exponent = ((*time_cnt) + 65535 - time_it) / (tau_it * 1000.0f);
	}
	else {
		exponent = ((*time_cnt) - time_it) / (tau_it * 1000.0f);
	}
	time_it = (*time_cnt);
	if(input_value >= output_value) direction = 1;
	else direction = 0;

	if(direction){
		U_we = input_value - output_value;
		U_value = U_we * (1.0f - pow(inv_euler, exponent));
		output_value += U_value;
	}
	else {
		U_we = output_value - input_value;
		U_value = U_we * (1.0f - pow(inv_euler, exponent));
		output_value -= U_value;
	}
	return output_value;
}
float PID::DerivativeTermCalculate(float input_value, uint16_t *time_cnt)
{
	static float inv_euler = 0.368f;
	static float output_value = 0.0f;
	static uint16_t time_dt = 0;
	static float previous_input_value = 0.0f;

	static float U_we = 0.0f;
	static float U_value = 0.0f;
	static float exponent = 0.0f;
	static float difference = 0.0f;

	if(time_dt > (*time_cnt)) {
		exponent = ((*time_cnt) + 65535 - time_dt) / (tau_dt * 1000.0f);
	}
	else {
		exponent = ((*time_cnt) - time_dt) / (tau_dt * 1000.0f);
	}

	time_dt = (*time_cnt);
	difference = input_value - previous_input_value;
	output_value += difference;

	if(output_value >= 0.0f) {
		U_value = output_value * (1.0f - pow(inv_euler, exponent));
		output_value -= U_value;
	}
	else {
		U_value = U_we * (1.0f - pow(inv_euler, exponent));
		output_value += U_value;
	}
	previous_input_value = input_value;
	return output_value;
}

