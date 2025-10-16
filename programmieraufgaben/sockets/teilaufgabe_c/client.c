#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define SERVER_IP "127.0.0.1"
#define PORT 12345


int main (void){

    struct sockaddr_in server;
    unsigned long addr;

    char buffer[1024] = {0};

    int client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0){
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&server, 0, sizeof(server));
    addr = inet_addr(SERVER_IP);
    memcpy((char*)&server.sin_addr, &addr, sizeof(addr));

    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);

    if (connect(client_socket, (struct sockaddr*)&server, sizeof(server)) < 0){
        perror("Connection failed");
        close(client_socket);
        exit(EXIT_FAILURE);
    }

    printf("Connected to %s:%d\n", SERVER_IP, PORT);

    while(1){
        // send command to server
        printf("> ");
        fgets(buffer, sizeof(buffer), stdin);
        send(client_socket, buffer, strlen(buffer), 0);

        if (strcmp(buffer, "QUIT\n") == 0){
            printf("Disconnected \n");
            exit(0);
        }

        read(client_socket, buffer, sizeof(buffer)-1);
        printf("%s" ,buffer);

        fd_set readfds;
        struct timeval tv;
        int rv;
        char recvbuf[1024];

        while(1){
            FD_ZERO(&readfds);
            FD_SET(client_socket, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 200000;

            rv = select(client_socket + 1, &readfds, NULL, NULL, &tv);
            if (rv <= 0) break;
            ssize_t n = recv(client_socket, recvbuf, sizeof(recvbuf) - 1, 0);
            if (n <= 0) {
                close(client_socket);               
                exit(0);
            }

            recvbuf[n] = '\0';
            printf("%s", recvbuf);
        }
    }
}