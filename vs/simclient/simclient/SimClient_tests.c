//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TESTS FOR SimClient MODULE.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <stdint.h>
#include "simclient.h"
#include <winsock2.h>
#include "controller.h"
#include "observer.h"
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#define SIMCLIENT_BUFFER_SIZE 512
typedef enum {
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


#define APP_NO_ERROR (0)
#define APP_ERROR (-2)

#define TCPCLIENT_START		((uint8_t)0x0A)
#define TCPCLIENT_END		((uint8_t)0xA0)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MOCK OBJECT DEFINITIONS.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int mock_TCPClient_write(SOCKET s, uint8_t* data, uint16_t len) {
	check_expected(data);
	check_expected(len);
	return mock_type(int);
}

int mock_TCPClient_read(SOCKET s, uint8_t* msg, uint16_t len) {
	// "Recevied message".
	uint8_t* mocked_msg = mock_type(uint8_t*);

	// Amount of bytes "recevied".
	int mocked_msg_len = mock_type(int);

	if (mocked_msg_len == SOCKET_ERROR) {
		return SOCKET_ERROR;
	}

	if (mocked_msg_len == APP_ERROR) {
		return APP_ERROR;
	}

	// Return "recevied" message.
	memcpy(msg, mocked_msg, min(mocked_msg_len, len));

	return min(mocked_msg_len, len);
}

int mock_TCPClient_connect(const char* host, const char* port) {
	return mock_type(int);
}

int mock_TCPClient_disconnect(SOCKET s, ReqType_e req) {
	return APP_NO_ERROR;
}

void Controller_step(SimData_s* x, SimData_s* sensed_x,
	SimData_s*observerd_x, float t, Controller_s* ctrl) {
	return;
}

void Observer_step(SimData_s* x, SimData_s* sensed_x, float t,
	Observer_s* obs) {
	return;
}

void test_SimClient_init_read_error_at_startup(void** state) {
	// Simclient will call TCPClient_write using this code as message.
	uint8_t expected_req = (uint8_t)ReqShutDownErr;

	// This value does not matter, as long as ptr is not null.
	Controller_s ctrl = { 0 };
	fcn_ctrl_step ctrl_step = Controller_step;
	
	// This value does not matter, as long as ptr is not null.
	Observer_s obs = { 0 };
	fcn_obs_step obs_step = Observer_step;

	will_return(mock_TCPClient_connect, ~INVALID_SOCKET);

	// Safe to pass a null ptr, it will not be used in mock fcn.
	will_return(mock_TCPClient_read, NULL);
	// Let the mock fcn know it must return an APP_ERROR.
	will_return(mock_TCPClient_read, APP_ERROR);

	// After receiving the invalid init message, SimClient
	// must send a ReqShutDownErr message to the server using
	// TCPClient_write.
	expect_memory(mock_TCPClient_write, data, &expected_req, 1);
	expect_value(mock_TCPClient_write, len, 1);
	will_return(mock_TCPClient_write, APP_NO_ERROR);

	// SimClient must return a NULL ptr as init was not successful.
	assert_ptr_equal(SimClient_init(&ctrl, Controller_step,
					 &obs, Observer_step), NULL);
}

void test_SimClient_init_ReqSimParams_error_at_startup(void **state) {
	uint8_t data = 0; // Data different from ReqSimParams(2) enum;

	// Simclient will call TCPClient_write using this code as message.
	uint8_t expected_req = (uint8_t)ReqShutDownErr;

	// This value does not matter, as long as ptr is not null.
	Controller_s ctrl = { 0 };
	fcn_ctrl_step ctrl_step = Controller_step;

	// This value does not matter, as long as ptr is not null.
	Observer_s obs = { 0 };
	fcn_obs_step obs_step = Observer_step;

	// We don't care so much about this. Assume connection successful and
	// return a valid socket.
	will_return(mock_TCPClient_connect, ~INVALID_SOCKET);

	// This is the hand-shake received from the client on connection.
	// Fake an invalid message. Just one byte.
	will_return(mock_TCPClient_read, &data);
	will_return(mock_TCPClient_read, 1);

	// After receiving the invalid init message, SimClient
	// must send a ReqShutDownErr message to the server using
	// TCPClient_write.
	expect_memory(mock_TCPClient_write, data, &expected_req, 1);
	expect_value(mock_TCPClient_write, len, 1);
	will_return(mock_TCPClient_write, APP_NO_ERROR);

	// SimClient must return a NULL ptr as init was not successful.
	assert_ptr_equal(SimClient_init(&ctrl, ctrl_step,
					 &obs, obs_step), NULL);
}

void test_SimClient_init_returns_valid_client(void** state) {
	uint8_t server_req_simparams = (uint8_t)ReqSimParams;
	uint8_t server_req_noerr = (uint8_t)ReqNoErr;

	uint8_t sim_params[] = {
		(uint8_t)ReqSimParams, // Code.
		0x01, // Has controller.
		0x00, 0x00, 0x4e, 0x20, // 20ms expressed as micro-secods.
		0x01, // Has observer.
		0x00, 0x00, 0x00, 0x64 // 100us.
	};
	SimClient_s* client;

	Controller_s ctrl;
	ctrl.ts = 0.02f;
	fcn_ctrl_step ctrl_step = Controller_step;

	Observer_s obs;
	obs.ts = 0.0001f;
	fcn_obs_step obs_step = Observer_step;

	// We don't care so much about this. Assume connection successful and
	// return a valid socket.
	will_return(mock_TCPClient_connect, ~INVALID_SOCKET);

	// This is the hand-shake received from the client on connection.
	// Fake the Init message. Just one byte.
	will_return(mock_TCPClient_read, &server_req_simparams);
	will_return(mock_TCPClient_read, 1);

	// After receiving the valid init message, SimClient
	// must send a ReqSimParams message to the server using
	// TCPClient_write.
	expect_memory(mock_TCPClient_write, data, sim_params, sizeof(sim_params));
	expect_value(mock_TCPClient_write, len, sizeof(sim_params));
	will_return(mock_TCPClient_write, APP_NO_ERROR);

	// After receiving sim params, the server returns a ReqNoErr code.
	will_return(mock_TCPClient_read, &server_req_noerr);
	will_return(mock_TCPClient_read, 1);

	// SimClient must return a NULL ptr as init was not successful.
	client = SimClient_init(&ctrl, ctrl_step, &obs, obs_step);
	assert_ptr_not_equal(client, NULL);
	free(client);
}

void test_SimClient_init_without_observer_returns_valid_client(void** state) {
	uint8_t server_req_simparams = (uint8_t)ReqSimParams;
	uint8_t server_req_noerr = (uint8_t)ReqNoErr;

	uint8_t sim_params[] = {
		(uint8_t)ReqSimParams, // Code.
		0x01, // Has controller.
		0x00, 0x00, 0x4e, 0x20, // 20ms expressed as micro-secods.
		0x00, // No observer.
		0x00, 0x00, 0x00, 0x00 // No timestep.
	};
	SimClient_s* client;

	Controller_s ctrl;
	ctrl.ts = 0.02f;
	fcn_ctrl_step ctrl_step = Controller_step;

	// We don't care so much about this. Assume connection successful and
	// return a valid socket.
	will_return(mock_TCPClient_connect, ~INVALID_SOCKET);

	// This is the hand-shake received from the client on connection.
	// Fake the Init message. Just one byte.
	will_return(mock_TCPClient_read, &server_req_simparams);
	will_return(mock_TCPClient_read, 1);

	// After receiving the valid init message, SimClient
	// must send a ReqSimParams message to the server using
	// TCPClient_write.
	expect_memory(mock_TCPClient_write, data, sim_params, sizeof(sim_params));
	expect_value(mock_TCPClient_write, len, sizeof(sim_params));
	will_return(mock_TCPClient_write, APP_NO_ERROR);

	// After receiving sim params, the server returns a ReqNoErr code.
	will_return(mock_TCPClient_read, &server_req_noerr);
	will_return(mock_TCPClient_read, 1);

	// SimClient must return a NULL ptr as init was not successful.
	client = SimClient_init(&ctrl, ctrl_step, NULL, NULL);
	assert_ptr_not_equal(client, NULL);
	free(client);
}

void test_SimClient_init_without_controller_returns_valid_client(void** state) {
	uint8_t server_req_simparams = (uint8_t)ReqSimParams;
	uint8_t server_req_noerr = (uint8_t)ReqNoErr;

	uint8_t sim_params[] = {
		(uint8_t)ReqSimParams, // Code.
		0x00, // No controller.
		0x00, 0x00, 0x00, 0x00, // No timestep.
		0x01, // Has observer.
		0x00, 0x00, 0x00, 0x64 // 100us.
	};
	SimClient_s* client;

	Observer_s obs;
	obs.ts = 0.0001f;
	fcn_obs_step obs_step = Observer_step;

	// We don't care so much about this. Assume connection successful and
	// return a valid socket.
	will_return(mock_TCPClient_connect, ~INVALID_SOCKET);

	// This is the hand-shake received from the client on connection.
	// Fake the Init message. Just one byte.
	will_return(mock_TCPClient_read, &server_req_simparams);
	will_return(mock_TCPClient_read, 1);

	// After receiving the valid init message, SimClient
	// must send a ReqSimParams message to the server using
	// TCPClient_write.
	expect_memory(mock_TCPClient_write, data, sim_params, sizeof(sim_params));
	expect_value(mock_TCPClient_write, len, sizeof(sim_params));
	will_return(mock_TCPClient_write, APP_NO_ERROR);

	// After receiving sim params, the server returns a ReqNoErr code.
	will_return(mock_TCPClient_read, &server_req_noerr);
	will_return(mock_TCPClient_read, 1);

	// SimClient must return a NULL ptr as init was not successful.
	client = SimClient_init(NULL, NULL, &obs, obs_step);
	assert_ptr_not_equal(client, NULL);
	free(client);
}

int SimClient_tests(void) {
	const struct CMUnitTest tests[] = {
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TESTS: SimClient_init.
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		cmocka_unit_test(test_SimClient_init_read_error_at_startup),
		cmocka_unit_test(test_SimClient_init_ReqSimParams_error_at_startup),
		cmocka_unit_test(test_SimClient_init_returns_valid_client),
		cmocka_unit_test(test_SimClient_init_without_controller_returns_valid_client),
		cmocka_unit_test(test_SimClient_init_without_observer_returns_valid_client),
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	};
	return cmocka_run_group_tests(tests, NULL, NULL);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// END OF TESTS: SimClient.
/*****************************************************************************/