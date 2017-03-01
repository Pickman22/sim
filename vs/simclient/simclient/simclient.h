#ifndef SIMCLIENT_H
#define SIMCLIENT_H

#include "tcpclient.h"
#include "controller.h"
#include "observer.h"

typedef struct SimClient SimClient_s;
typedef struct Observer Observer_s;
typedef struct Controller Controller_s;
typedef struct SimData SimData_s;
typedef int ReqType_e;

typedef void(*fcn_ctrl_step)(SimData_s* x, SimData_s* sensed_x,
	SimData_s*observerd_x, float t, Controller_s* ctrl);

typedef void(*fcn_obs_step)(SimData_s* x, SimData_s* sensed_x, float t,
	Observer_s* obs);

SimClient_s* SimClient_init(Controller_s* ctrl,
	fcn_ctrl_step ctrl_step, Observer_s* obs, fcn_obs_step obs_step);

int SimClient_step(SimClient_s* client);

int SimClient_stop(SimClient_s* client, ReqType_e req);

#endif // SIMCLIENT_H