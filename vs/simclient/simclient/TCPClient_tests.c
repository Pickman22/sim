#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <stdio.h>

/******************************************************************************
TESTS: TCPClient.
******************************************************************************/
#include "tcpclient.h"

#define TCPCLIENT_START		((uint8_t)0x0A)
#define TCPCLIENT_END		((uint8_t)0xA0)
#define TCPCLIENT_SHUTDOWN	((uint8_t)0xAF)
#define TCPCLIENT_ERROR		((uint8_t)0xE0)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MOCK OBJECT DEFINITIONS.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int mock_send(SOCKET s, uint8_t* msg, int len, int flags) {
	check_expected(len);
	check_expected(msg);
	return mock_type(int);
}

int mock_recv(SOCKET s, uint8_t* msg, int len, int flags) {
	uint8_t* mock_msg = mock_type(uint8_t*);
	int recv_bytes_len = mock_type(int);

	if (recv_bytes_len == SOCKET_ERROR) {
		return SOCKET_ERROR;
	}

	memcpy(msg, mock_msg, min(recv_bytes_len, len));
	return min(recv_bytes_len, len);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TESTS FOR TCPClient MODULE.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static void test_TCPClient_write_pack_msg(void **state) {
	SOCKET s = ~INVALID_SOCKET;
	char msg[] = "Hello, world!";
	uint8_t len = sizeof(msg);
	char* packed_msg = malloc((len + 4) * sizeof(char));
	packed_msg[0] = TCPCLIENT_START;
	packed_msg[1] = 0;
	packed_msg[2] = len;
	memcpy(&packed_msg[3], msg, len);
	packed_msg[len + 3] = TCPCLIENT_END;

	expect_value(mock_send, len, len + 4);
	expect_string(mock_send, msg, packed_msg);

	will_return(mock_send, len + 4);
	assert_int_equal(TCPClient_write(s, msg, len), APP_NO_ERROR);
}

static void test_TCPClient_write_returns_socket_error(void **state) {
	SOCKET s = ~INVALID_SOCKET;
	char msg[] = "Hello, world!";
	uint8_t len = sizeof(msg);

	expect_value(mock_send, len, len + 4);
	expect_any(mock_send, msg);
	will_return(mock_send, SOCKET_ERROR);
	assert_int_equal(TCPClient_write(s, msg, len), SOCKET_ERROR);
}

static void test_TCPClient_write_returns_app_error(void **state) {
	SOCKET s = ~INVALID_SOCKET;
	char msg[] = "Hello, world";
	uint8_t len = sizeof(msg);

	expect_value(mock_send, len, len + 4);
	expect_any(mock_send, msg);

	// Returns a different length than that of the message.
	// This must result in a APP_ERROR return value.
	will_return(mock_send, len + 1);
	assert_int_equal(TCPClient_write(s, msg, len), APP_ERROR);
}

static void test_TCPClient_read_returns_socket_error(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	const uint8_t expected_msg[] = "Hello, world!";
	const uint8_t header[] = { TCPCLIENT_START, 0, sizeof(expected_msg) };
	const uint8_t end[] = { TCPCLIENT_END };

	uint8_t buffer[sizeof(expected_msg)];

	will_return(mock_recv, NULL);
	will_return(mock_recv, SOCKET_ERROR);

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), SOCKET_ERROR);
}

static void test_TCPClient_read_short_message(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	const uint8_t expected_msg[] = "Hello, world!";
	const uint8_t header[] = { TCPCLIENT_START, 0, sizeof(expected_msg) };
	const uint8_t end[] = { TCPCLIENT_END };

	uint8_t buffer[sizeof(expected_msg)];
	memset(buffer, 0, sizeof(buffer));

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	will_return(mock_recv, expected_msg);
	will_return(mock_recv, sizeof(expected_msg));

	will_return(mock_recv, end);
	will_return(mock_recv, sizeof(end));

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), sizeof(expected_msg));
	assert_memory_equal(buffer, expected_msg, sizeof(expected_msg));

}

static void test_TCPClient_read_long_message(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	char expected_msg[0xffff];
	memset(expected_msg, 0x10, sizeof(expected_msg));

	const uint8_t header[] = { TCPCLIENT_START, 0xff, 0xff };
	const uint8_t end[] = { TCPCLIENT_END };

	uint8_t buffer[sizeof(expected_msg)];
	memset(buffer, 0, sizeof(buffer));

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	will_return(mock_recv, expected_msg);
	will_return(mock_recv, sizeof(expected_msg));

	will_return(mock_recv, end);
	will_return(mock_recv, sizeof(end));

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), sizeof(expected_msg));
	assert_memory_equal(buffer, expected_msg, sizeof(expected_msg));

}

static void test_TCPClient_read_no_start_byte(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	const uint8_t expected_msg[] = "Hello, world!";
	const uint8_t header[] = { 0, 0, sizeof(expected_msg) }; // No start.
	const uint8_t end[] = { TCPCLIENT_END };
	uint8_t buffer[sizeof(expected_msg)];
	memset(buffer, 0, sizeof(buffer));

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), APP_ERROR);
}

static void test_TCPClient_read_no_end_byte(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	const uint8_t expected_msg[] = "Hello, world!";
	const uint8_t header[] = { TCPCLIENT_START, 0, sizeof(expected_msg) };
	const uint8_t end[] = { 0 }; // No end.
	uint8_t buffer[sizeof(expected_msg)];
	memset(buffer, 0, sizeof(buffer));

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	will_return(mock_recv, expected_msg);
	will_return(mock_recv, sizeof(expected_msg));

	will_return(mock_recv, end);
	will_return(mock_recv, sizeof(end));

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), APP_ERROR);
}

static void test_TCPClient_read_buffer_len_too_small(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	const uint8_t expected_msg[] = "Hello, world!";
	const uint8_t header[] = { TCPCLIENT_START, 0, sizeof(expected_msg) };
	const uint8_t end[] = { TCPCLIENT_END };

	uint8_t buffer[sizeof(expected_msg)];
	memset(buffer, 0, sizeof(buffer));

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	// Fake the size of the buffer to force an APP_ERROR.
	assert_int_equal(TCPClient_read(s, buffer, 1), APP_ERROR);

}

static void test_TCPClient_read_large_divided_message(void **state) {
	SOCKET s = ~INVALID_SOCKET;

	uint8_t expected_msg[0xffff];
	memset(expected_msg, 0x10, sizeof(expected_msg));

	const uint8_t header[] = { TCPCLIENT_START, 0xff, 0xff };
	const uint8_t end[] = { TCPCLIENT_END };

	uint8_t buffer[sizeof(expected_msg)];
	memset(buffer, 0, sizeof(buffer));

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	// Send the first half of the mssage.
	will_return(mock_recv, expected_msg);
	will_return(mock_recv, 0x7fff);

	// Send the remaining half of the message.
	will_return(mock_recv, expected_msg + 0x7fff);
	will_return(mock_recv, 0x8000);

	will_return(mock_recv, end);
	will_return(mock_recv, sizeof(end));

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), sizeof(expected_msg));
	assert_memory_equal(buffer, expected_msg, sizeof(expected_msg));
}

void test_TCPClient_read_zero_bytes_return_app_error(void **state) {
	SOCKET s = ~INVALID_SOCKET;
	uint8_t header[] = { TCPCLIENT_START, 0, 0 }; // Zero length data.
	uint8_t buffer[64] = { 0 }; // Don't care about this buffer.

	will_return(mock_recv, header);
	will_return(mock_recv, sizeof(header));

	assert_int_equal(TCPClient_read(s, buffer, sizeof(buffer)), APP_ERROR);
}

int TCPClient_tests(void) {
	const struct CMUnitTest tests[] = {
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TESTS: TCPClient_write.
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		cmocka_unit_test(test_TCPClient_write_pack_msg),
		cmocka_unit_test(test_TCPClient_write_returns_socket_error),
		cmocka_unit_test(test_TCPClient_write_returns_app_error),
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TESTS: TCPClient_read.
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		cmocka_unit_test(test_TCPClient_read_returns_socket_error),
		cmocka_unit_test(test_TCPClient_read_short_message),
		cmocka_unit_test(test_TCPClient_read_long_message),
		cmocka_unit_test(test_TCPClient_read_no_start_byte),
		cmocka_unit_test(test_TCPClient_read_no_end_byte),
		cmocka_unit_test(test_TCPClient_read_buffer_len_too_small),
		cmocka_unit_test(test_TCPClient_read_large_divided_message),
		cmocka_unit_test(test_TCPClient_read_zero_bytes_return_app_error),
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	};
	return cmocka_run_group_tests(tests, NULL, NULL);
}