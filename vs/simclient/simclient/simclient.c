#include "simclient.h"
#include <assert.h>
#include "utils.h"

#define SIMCLIENT_BUFFER_SIZE 512
#define SIMCLIENT_SCALING ((uint32_t)1000)

#ifdef RUN_TESTS
#define TCPClient_connect    mock_TCPClient_connect
#define TCPClient_disconnect mock_TCPClient_disconnect
#define TCPClient_write      mock_TCPClient_write
#define TCPClient_read       mock_TCPClient_read
#endif

typedef enum{
	ReqShutDownErr = -2,
	ReqErr = -1,
	ReqNoErr = 0,
	ReqShutDownNoErr,
	ReqSimParams,
	ReqCtrlStep,
	ReqCtrlSignals,
	ReqObsStep,
	ReqObsSignals,
} ReqType_e;

typedef struct SimClient {
	SOCKET _socket;
	Controller_s* ctrl;
	fcn_ctrl_step ctrl_step;
	Observer_s* obs;
	fcn_obs_step obs_step;
	uint8_t is_connected;
	uint8_t _buffer[SIMCLIENT_BUFFER_SIZE];
}SimClient_s;

#define SERVER_IP "localhost"
#define SERVER_PORT "9090"

SimClient_s* SimClient_init(Controller_s * ctrl,
	fcn_ctrl_step ctrl_step, Observer_s * obs, fcn_obs_step obs_step)
{
	uint32_t u32_ts;
	SimClient_s* client;
	assert(ctrl || obs);
	client = malloc(sizeof(SimClient_s));
	memset(client, 0, sizeof(SimClient_s));

	client->ctrl = ctrl;
	client->obs = obs;
	client->ctrl_step = ctrl_step;
	client->obs_step = obs_step;

	SOCKET socket = TCPClient_connect(SERVER_IP, SERVER_PORT);
	if (socket == INVALID_SOCKET) {
		return NULL;
	}
	client->_socket = socket;
	client->is_connected = TRUE;

	if(TCPClient_read(client->_socket, client->_buffer, SIMCLIENT_BUFFER_SIZE) < APP_NO_ERROR) {
		SimClient_stop(client, ReqShutDownErr);
		return NULL;
	}

	if ((ReqType_e)client->_buffer[0] != ReqSimParams) {
		SimClient_stop(client, ReqShutDownErr);
		return NULL;
	}

	memset(client->_buffer, 0, SIMCLIENT_BUFFER_SIZE);
	client->_buffer[0] = (uint8_t)ReqSimParams;
	if (client->ctrl) {
		assert(ctrl_step);
		assert(client->ctrl->ts > 0);
		client->_buffer[1] = 1;
		u32_ts = f_to_scaled_u32(client->ctrl->ts, 1e6);
		u32_to_u8_arr(u32_ts, &client->_buffer[2]);
	}
	if (client->obs) {
		assert(obs_step);
		assert(client->obs->ts > 0);
		client->_buffer[6] = 1;
		u32_ts = f_to_scaled_u32(client->obs->ts, 1e6);
		u32_to_u8_arr(u32_ts, &client->_buffer[7]);
	}
	if (TCPClient_write(client->_socket, client->_buffer, 11) < APP_NO_ERROR) {
		SimClient_stop(client, ReqShutDownErr);
		return NULL;
	}
	memset(client->_buffer, 0, SIMCLIENT_BUFFER_SIZE);
	if (TCPClient_read(client->_socket, client->_buffer, SIMCLIENT_BUFFER_SIZE) < APP_NO_ERROR) {
		SimClient_stop(client, ReqShutDownErr);
		return NULL;
	}
	if (client->_buffer[0] != ReqNoErr) {
		SimClient_stop(client, ReqShutDownErr);
		return NULL;
	}
	return client;
}

int SimClient_run(SimClient_s* client) {
	assert(client);
	while (client->is_connected) {
		// Get command and args.
		if (TCPClient_read(client->_socket, client->_buffer, SIMCLIENT_BUFFER_SIZE) < APP_NO_ERROR) {
			SimClient_stop(client, ReqShutDownErr);
			return APP_ERROR;
		}

		// Executed command if valid.
		switch (client->_buffer[0]) {
			case ReqCtrlStep:
				break;

			case ReqCtrlSignals:
				break;

			case ReqObsStep:
				break;

			case ReqObsSignals:
				break;

			default:
				// Handle error.
				client->_buffer[0] = (uint8_t)ReqErr;
				TCPClient_write(client->_socket, client->_buffer, sizeof(uint8_t));
				break;
		}
	}

	return APP_ERROR;
}

int SimClient_step(SimClient_s* client) {
	assert(client);
	return APP_ERROR;
}

int SimClient_stop(SimClient_s* client, ReqType_e req) {
	assert(client);
	uint8_t data = req;
	int ret;
	if ((client->_socket == INVALID_SOCKET) || !client->is_connected) {
		ret = APP_ERROR;
	}
	else if (TCPClient_write(client->_socket, &data, 1) != APP_NO_ERROR) {
		ret = APP_ERROR;
	}
	else {
		ret = APP_NO_ERROR;
	}
	free(client);
	return ret;
}
