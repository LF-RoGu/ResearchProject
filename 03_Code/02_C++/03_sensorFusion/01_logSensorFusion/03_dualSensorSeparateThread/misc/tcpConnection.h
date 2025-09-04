#include <iostream>
#include <string>
#include <cstring>      // for memset
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h> // for sockaddr_in
#include <arpa/inet.h>  // for inet_addr
#include <unistd.h>     // for close

int setupTcpSocket(const std::string& ip, int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "[ERROR] Failed to create socket\n";
        return -1;
    }

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sockfd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0)
    {
        std::cerr << "[ERROR] Bind failed\n";
        close(sockfd);
        return -1;
    }

    if (listen(sockfd, 1) < 0)
    {
        std::cerr << "[ERROR] Listen failed\n";
        close(sockfd);
        return -1;
    }

    std::cout << "[INFO] Waiting for Python client to connect on " << ip << ":" << port << "...\n";

    int connfd = accept(sockfd, nullptr, nullptr);  // ⬅️ BLOCKS here until client connects
    if (connfd < 0)
    {
        std::cerr << "[ERROR] Accept failed\n";
        close(sockfd);
        return -1;
    }

    std::cout << "[INFO] Client connected successfully\n";
    return connfd;
}

inline void closeTcpSocket(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}
