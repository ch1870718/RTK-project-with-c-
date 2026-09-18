#pragma once
#include<Windows.h>
bool OpenSocket(SOCKET& sock, const char ip[], const unsigned short port);
void CloseSocket(SOCKET& sock);