/* 
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the	
 * specific language governing permissions and limitations
 * under the License. 
 */
#include "PID.h"

#include <stdlib.h>
#include <stdio.h>

PID_t *pid_create_empty(void)
{
	return malloc(sizeof(PID_t));
}

PID_t *pid_create(float Kp, float Ki, float Kd, float i_lim, float d_filt_gain)
{
	PID_t *pid = pid_create_empty();
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->i_lim = i_lim;
	pid->d_filt_gain = d_filt_gain;
	pid->i_term = 0.0;
	pid->err_prev = 0.0;
	pid->d_term_prev = 0.0;
	return pid;
}

void pid_process(PID_t *pid, float dt, float err, float set, float *out)
{
	float p_term, i_term, d_term;
	
	p_term = pid->Kp*err;
	i_term += pid->Ki*dt*err;
	d_term = pid->Kd*(err-(pid->err_prev))/dt;
	
	
	// Integrator windup guard
	if(i_term > (pid->i_lim)) {
		i_term = pid->i_lim;
	}
	else if(i_term < -(pid->i_lim)) {
		i_term = -(pid->i_lim);
	}
	
	// Derivative term LPF
	d_term = (1.0-(pid->d_filt_gain))*d_term - (pid->d_filt_gain)*(pid->d_term_prev);
	
	//Store stuff
	pid->d_term_prev = d_term;
	pid->err_prev = err;
	
	*out = p_term + i_term + d_term;
	
}

void pid_reset(PID_t *pid)
{
	pid->i_term = 0.0;
	pid->err_prev = 0.0;
}

void pid_set_gains(PID_t *pid, float Kp, float Ki, float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

void pid_set_i_lim(PID_t *pid, float i_lim)
{
	pid->i_lim = i_lim;
}

void pid_set_d_filt_gain(PID_t *pid, float d_filt_gain)
{
	pid->d_filt_gain = d_filt_gain;
}

int main(void) {
	printf("BEGIN\n");
	PID_t *pid = pid_create(1.0, 1.0, 1.0, 1.0, 0);
	printf("%f\n",pid->Kp);
	float out=0.0;
	while(1) {
		pid_process(pid, 0.1, 1.0, 0.0, &out);
		printf("%f\n",out);
	}
}