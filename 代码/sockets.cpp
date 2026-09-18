#include <stdio.h>
#include "sockets.h"

#pragma comment(lib, "WS2_32.lib")
#pragma warning(disable:4996)



// 打开Socket连接
bool OpenSocket(SOCKET& sock, const char ip[], const unsigned short port) {
    WSADATA wsaData;
    SOCKADDR_IN serverAddr;

    // 初始化Winsock
    if (WSAStartup(MAKEWORD(1, 1), &wsaData) != 0) return false;

    // 创建Socket
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET) {
        WSACleanup();
        return false;
    }

    // 配置服务器地址
    serverAddr.sin_addr.S_un.S_addr = inet_addr(ip);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    // 连接服务器
    if (connect(sock, (SOCKADDR*)&serverAddr, sizeof(SOCKADDR)) != 0) {
        closesocket(sock);
        WSACleanup();
        return false;
    }

    return true;
}

// 关闭Socket连接
void CloseSocket(SOCKET& sock) {
    if (sock != INVALID_SOCKET) {
        closesocket(sock);
    }
    WSACleanup();
}