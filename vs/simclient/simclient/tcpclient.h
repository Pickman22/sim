#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <winsock2.h>
#include <stdint.h>

#define APP_NO_ERROR (0)
// Error code (-1) reserved to detect a SOCKET_ERROR
#define APP_ERROR (-2)

SOCKET TCPClient_connect(const char * const server_ip, const char * const port);

int TCPClient_read(SOCKET sock, uint8_t* data, uint16_t len);

int TCPClient_write_code(SOCKET sock, uint8_t code);

int TCPClient_write(SOCKET sock, const uint8_t * const bytes, uint16_t len);

int TCPClient_disconnect(SOCKET sock);

#endif // TCPCLIENT_H