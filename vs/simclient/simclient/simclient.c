#include "simclient.h"
#include <assert.h>

#define SIMCLIENT_BUFFER_SIZE 512

#define SIMCLIENT_SCALING ((uint32_t)1000)

#define SIMCLIENT_ERROR ((uint8_t)0x0B)
#define SIMCLIENT_SHUTDOWN ((uint8_t)0x0C)
#define SIMCLIENT_NO_CODE ((uint8_t)0x00)

#define CTRL_GET_ERROR_CMD 0x13
#define CTRL_GET_OUTPUT_CMD 0x10
#define CTRL_GET_FEEDBACK_CMD 0x21
#define CTRL_GET_TARGET_CMD 0x70
#define CTRL_STEP_CMD 0x80

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

int _check_elements(SimClient_s* client) {
	assert(client);
	if ((client->ctrl && !client->ctrl_step) || (client->ctrl_step && !client->ctrl)) {
		// If a controller is provided, then the step function must be provided too.
		return APP_ERROR;
	}

	if ((client->obs && !client->obs_step) || (client->obs_step && !client->obs)) {
		// If an observer is provided, then the step function must be provided too.
		return APP_ERROR;
	}

	if (!client->obs && !client->ctrl) {
		// A contrroller or an observer must be provided. Otherwise, there is no point to
		// run a simulation in external mode.
		return APP_ERROR;
	}
	return APP_NO_ERROR;
}

static int _recv_sim_data(SimClient_s* client) {
	// Call TCPClient_read and parse data.
	return APP_NO_ERROR;
}

static int _send_sim_data(SimClient_s* client, uint8_t* data, uint16_t len) {
	// Call TCPClient_write and write data.
	return APP_NO_ERROR;
}

static int _to_bytes(float var, uint8_t data[4]) {
	assert(data);
	int32_t bytes = (int32_t)(var * SIMCLIENT_SCALING);
	uint8_t i;
	for (i = 0; i < 4; ++i) {
		data[i] = (uint8_t)((bytes >> (8 * i)));
	}
}

int SimClient_init(SimClient_s* client, Controller_s * ctrl,
	fcn_ctrl_step ctrl_step, Observer_s * obs, fcn_obs_step obs_step)
{
	assert(client);
	memset(client, 0, sizeof(SimClient_s));

	client->ctrl = ctrl;
	client->obs = obs;
	client->ctrl_step = ctrl_step;
	client->obs_step = obs_step;

	if (_check_elements(client) == APP_ERROR) {
		return APP_ERROR;
	}

	SOCKET socket = TCPClient_connect(SERVER_IP, SERVER_PORT);
	if (socket == INVALID_SOCKET) {
		return APP_ERROR;
	}
	client->_socket = socket;
	client->is_connected = TRUE;
	return 0;
}

int SimClient_run(SimClient_s* client) {
	assert(client);
	if ((_check_elements(client) == APP_ERROR) || !client->is_connected) {
		return APP_ERROR;
	}
	while (client->is_connected) {
		// Is step all we need?
		SimClient_step(client);
	}
}

int SimClient_step(SimClient_s* client) {
	int ret;
	if (_check_elements(client) == APP_ERROR) {
		return APP_ERROR;
	}

	if (!client->is_connected) {
		return APP_ERROR;
	}

	ret = TCPClient_read(client->_socket, client->_buffer, SIMCLIENT_BUFFER_SIZE);
	if (ret != APP_NO_ERROR) {
		SimClient_stop(client->_socket, SIMCLIENT_ERROR);
		client->_socket = INVALID_SOCKET;
		return APP_ERROR;
	}

	switch(client->_buffer[0]) {
		case CTRL_GET_OUTPUT_CMD:

			break;

		case CTRL_GET_ERROR_CMD:

			break;

		case CTRL_GET_FEEDBACK_CMD:

			break;

		case CTRL_GET_TARGET_CMD:

			break;

		case CTRL_STEP_CMD:
			if (client->ctrl && client->ctrl_step) {
				// Step the controller!
			}
			else {
				SimClient_stop(client, SIMCLIENT_ERROR);
				return APP_ERROR;
			}
			break;

		default:
			// Unknown command! Stop simulation.
			return APP_ERROR;
	}

	// Determine if command is valid: get controller output only if a controller is available.
	// Otherwise, return APP_ERROR and finish the simulation. Let the other side that we're shutting down.

	// Execute command.

	// Start over.

	if (client->ctrl) {
		// Step Controller.
	}

	if (client->obs) {
		// Step Observer.
	}

	// Write Simulation message.
	return APP_NO_ERROR;
}

int SimClient_stop(SimClient_s* client, uint8_t code) {
	int ret;
	assert(client);
	if ((client->_socket == INVALID_SOCKET) || !client->is_connected) {
		return APP_ERROR;
	}
	if (code != SIMCLIENT_NO_CODE) {
		TCPClient_write_code(client->_socket, code);
	}
	if (TCPClient_disconnect(client->_socket) == SOCKET_ERROR) {
		ret = SOCKET_ERROR;
	}
	else {
		ret = APP_NO_ERROR;
	}
	return ret;
}
