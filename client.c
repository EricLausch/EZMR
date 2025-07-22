#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string.h>
#include <stdio.h>

int main (void){

    struct sockaddr_in server;
    unsigned long addr;

    char* msg = "Hello from client";
    char buffer[1024] = {0};

    int client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0) printf("Error creating the socket\n");
    //else printf ("Client socket created succesfully\n");

    memset(&server, 0, sizeof(server));

    addr = inet_addr("141.57.57.204");
    memcpy((char*)&server.sin_addr, &addr, sizeof(addr));

    server.sin_family = AF_INET;
    server.sin_port = htons(12345);

    if (connect(client_socket, (struct sockaddr*)&server, sizeof(server)) < 0){
        printf ("Connection error\n");
    }

    for (;;){
        send(client_socket, msg, strlen(msg), 0);
        read(client_socket, buffer, 1024 - 1);
        printf("%s\n" ,buffer);

        //close(client_socket);
    }
}