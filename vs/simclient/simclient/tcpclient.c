#include "tcpclient.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <ws2tcpip.h>
#include <stdint.h>

#pragma comment(lib, "Ws2_32.lib")

#define TCPCLIENT_START		((uint8_t)0x0A)
#define TCPCLIENT_END		((uint8_t)0xA0)
#define TCPCLIENT_SHUTDOWN	((uint8_t)0xAF)
#define TCPCLIENT_ERROR		((uint8_t)0xE0)

static void _pack_data(char const * const data, char*  packed_data, uint16_t data_size) {
	assert(data);
	assert(packed_data);
	assert(data_size > 0);
	packed_data[0] = TCPCLIENT_START;
	packed_data[1] = (uint8_t)((data_size >> 8) & 0xff);
	packed_data[2] = (uint8_t)(data_size & 0xff);
	memcpy((void*)&packed_data[3], (void*)data, data_size);
	packed_data[data_size + 3] = TCPCLIENT_END;
}

SOCKET TCPClient_connect(const char * const server_ip, const char * const port) {
	WSADATA ws_data;
	struct addrinfo hints, *res;
	struct sockaddr_in *server_addr;
	SOCKET sock = INVALID_SOCKET;
	char addr_str[INET_ADDRSTRLEN];

	// Initialize to invalid. If we fail without reaching the end of the function,
	// then the user can test for the invalid socket as well as the return error code.
	if (WSAStartup(MAKEWORD(2, 2), &ws_data) != 0) {
		printf("WSAStartup failed.\n\r");
		return INVALID_SOCKET;
	}
	else {
		printf("WSAStartup initialized correctly.\n\r");
	}

	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock == INVALID_SOCKET) {
		printf("Could not create socket.\n\r");
		WSACleanup();
		return INVALID_SOCKET;
	}
	else {
		printf("Socket created succesfully.\n\r");
	}

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if (getaddrinfo(server_ip, port, &hints, &res) != 0) {
		printf("getaddinfo falied.\n\r");
		WSACleanup();
		freeaddrinfo(res);
		return INVALID_SOCKET;
	}

	// There should only be one IP for localhost.
	assert(!res->ai_next);
	printf("IPv4 assumed.\n\r");
	server_addr = (struct sockaddr_in*)res->ai_addr;
	inet_ntop(AF_INET, (void*)&server_addr->sin_addr, addr_str, sizeof(addr_str));
	printf("\tIPv4 host address: %s\n\r", addr_str);
	
	if (connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
		printf("Could not connect to server.\n\r");
		freeaddrinfo(res);
		return INVALID_SOCKET;
	}
	freeaddrinfo(res);
	return sock;
}

int TCPClient_read(SOCKET sock, uint8_t* data, uint16_t len) {
	assert(data);
	assert(len > 0);
	assert(sock != INVALID_SOCKET);
	int32_t ret = 0;
	int32_t bytes_left = 0;
	uint16_t actual_sz;
	uint8_t last_byte;
	memset(data, 0, len);
	ret = recv(sock, data, 3, 0);
	if ((data[0] != TCPCLIENT_START) || (ret != 3)) {
		return APP_ERROR;
	}
	actual_sz = (data[1] << 8) | data[2];
	if (actual_sz > len) {
		return APP_ERROR;
	}
	bytes_left = actual_sz;
	ret = 0;
	while (bytes_left > 0) {
		ret = recv(sock, data + ret, bytes_left, 0);
		if (ret == SOCKET_ERROR) {
			return SOCKET_ERROR;
		}
		bytes_left -= ret;
	}
	ret = recv(sock, &last_byte, 1, 0);
	if (ret == SOCKET_ERROR) {
		return SOCKET_ERROR;
	}
	if (last_byte != TCPCLIENT_END) {
		return APP_ERROR;
	}
	return actual_sz;
}

int TCPClient_write_code(SOCKET sock, uint8_t code) {
	char cmd[] = { TCPCLIENT_START, 0x00, 0x01, code };
	int ret;
	if (sock == INVALID_SOCKET) {
		return APP_ERROR;
	}
	ret = send(sock, cmd, sizeof(cmd), 0);
	if (ret == SOCKET_ERROR) {
		ret = SOCKET_ERROR;
	}
	else if (ret != sizeof(cmd)) {
		ret = APP_ERROR;
	}
	else {
		ret = APP_NO_ERROR;
	}
	return ret;
}

int TCPClient_write(SOCKET sock, const uint8_t * const data, uint16_t len) {
	assert(data);
	assert(len > 0);
	assert(sock != INVALID_SOCKET);
	int ret = -1;
	unsigned int sz = (len + 4) * sizeof(uint8_t);
	uint8_t* packed_data = malloc(sz);
	if (packed_data) {
		_pack_data(data, packed_data, len);
		ret = send(sock, packed_data, sz, 0);
		if (ret == SOCKET_ERROR) {
			ret = SOCKET_ERROR;
		}
		else if (ret != sz) {
			ret = APP_ERROR;
		}
		else {
			ret = APP_NO_ERROR;
		}
		free(packed_data);
	}
	return ret;
}

int TCPClient_disconnect(SOCKET sock) {
	int ret = APP_ERROR;
	assert(sock != INVALID_SOCKET);
	if (closesocket(sock) != SOCKET_ERROR) {
		ret = APP_NO_ERROR;
	}
	else {
		ret = SOCKET_ERROR;
	}
	WSACleanup();
	return ret;
}


