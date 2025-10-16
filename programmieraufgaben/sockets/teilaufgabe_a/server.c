#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#define PORT 12345

void chat(int);

int main (void){

    struct sockaddr_in server, client;
    int new_sock, pid;
    socklen_t len;

    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) printf("Error creating the socket\n");

    memset( &server, 0, sizeof (server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    server.sin_port = htons(PORT);

    if(bind(server_socket, (struct sockaddr*)&server, sizeof(server)) < 0){
        printf ("Binding error\n");
    }

    listen(server_socket, 3);
    len = sizeof(client);

    printf("Server startet on port %d\n", PORT);

    while (1) {

        new_sock = accept(server_socket,(struct sockaddr*)&client, &len);
        if(new_sock < 0){
            printf("Error on accept\n");
            continue;
        }
        
        chat(new_sock);
        close(new_sock); 
    }
}

void chat (int sock){
    
    char* msg;
    char buffer[1024] = {0};

    while(1){
        read(sock, buffer, sizeof(buffer)-1);
         
        if (strncmp(buffer, "PING", 4) == 0){
            msg = "PONG\n";
            send(sock, msg, strlen(msg),0);
        }

        printf("%s",buffer);
        memset(buffer, 0, sizeof(buffer));
    }
}


