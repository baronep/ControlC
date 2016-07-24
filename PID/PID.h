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
#ifndef PID_H_
#define PID_H_

typedef struct {
		float Kp, Ki, Kd;
		float i_term, i_lim;
		float err_prev, d_term_prev;
		float setpoint, output;
		float d_filt_gain;
	} PID_t;
	
PID_t *pid_create_empty(void);
PID_t *pid_create(float Kp, float Ki, float Kd, float i_lim, float d_filt_gain);
void pid_process(PID_t *pid, float dt, float err, float set, float *out);
void pid_reset(PID_t *pid);
void pid_set_gains(PID_t *pid, float Kp, float Ki, float Kd);
void pid_set_i_lim(PID_t *pid, float i_lim);
void pid_set_d_filt_gain(PID_t *pid, float d_filt_gain);

#endif