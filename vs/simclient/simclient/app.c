#include <stdio.h>
#include <stdint.h>
#include "tcpclient.h"

int app()
{
	char msg[] = "Hello, from python!";
	char buffer[128];
	int32_t sz;
	SOCKET sock = TCPClient_connect("localhost", "9090");
	if (sock == INVALID_SOCKET) {
		printf("Could not connect to server.\n\r");
		exit(1);
	}
	TCPClient_write(sock, msg, sizeof(msg));
	sz = TCPClient_read(sock, buffer, sizeof(buffer));
	if (sz < 0) {
		printf("An error ocurred reading the data.\n\r");
		if (sz == APP_ERROR) {
			printf("Application error!\n\r");
		}
		TCPClient_disconnect(sock);
		(void)getchar();
		exit(1);
	}
	buffer[sz] = '\0';
	printf("Recevied %d bytes.\n\r", sz);
	printf("Received message: %s\n\r", buffer);
	TCPClient_disconnect(sock);
	(void)getchar();
	return 0;
}