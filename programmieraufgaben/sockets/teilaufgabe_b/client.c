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
        perror("Connection error");
        close(client_socket);
        exit(EXIT_FAILURE);
    }

    printf("Connected to %s:%d\n", SERVER_IP, PORT);
    printf("available commands:\n");
    printf("PING: send ping to server\n");
    printf("QUIT: disconnect from server\n");
    printf("---------------------\n");


    while(1){

        printf("> ");
        fgets(buffer, sizeof(buffer), stdin);

        send(client_socket, buffer, strlen(buffer), 0);
        buffer[strcspn(buffer, "\n")] = 0;

        if (strcmp(buffer, "QUIT") == 0){
            printf("Disconnected \n");
            exit(0);
        }

        read(client_socket, buffer, sizeof(buffer)-1);
        printf("%s" ,buffer);
    }
}