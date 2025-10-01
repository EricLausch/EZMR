#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//
#define ENTRYLEN 32

//TBD
// Server socket beenden bei ^C

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
    server.sin_port = htons(12345);

    if(bind(server_socket, (struct sockaddr*)&server, sizeof(server)) < 0){
        printf ("Binding error\n");
    }

    listen(server_socket, 3);
    len = sizeof(client);

    while (1) {

        new_sock = accept(server_socket,(struct sockaddr*)&client, &len);
        if(new_sock < 0) printf("Error on accept\n");

        pid = fork();
        if (pid == 0){
            close(server_socket);
            chat(new_sock);
            exit(0);
        }
    }
}

void chat (int sock){

    int j = 0;

    char* msg ="Hello from server\n";
    char* gb = "GOODBYE CLIENT!\n";
    char buffer[1024] = {0};

    char data[5][ENTRYLEN] = {"Banane", "Gulash", "Bier"};
    char tempdata[5][ENTRYLEN] = {0};

    while(1){
        read(sock, buffer, 1024-1);

        if (strncmp(buffer, "EXIT", 4) == 0){
            send(sock, gb, strlen(gb),0);
            exit(0);
        }
        else if (strncmp(buffer, "GETLIST", 6) == 0){
            for (int i = 0; i <= (sizeof(data)/ENTRYLEN); i++)
            {
                if (data[i][0] != '\0'){
                    send(sock, data[i], strlen(data[i]),0);
                    send(sock, "\n", strlen("\n"),0);
                    memset(buffer, 0, sizeof buffer);
                }
            }        
        } 
        else{
            printf("%s",buffer);

            if (strncmp(buffer, "UPDATELIST",9) == 0){
                memcpy(data, tempdata, sizeof(tempdata));
                j = 0;
            }
            else{
                if (j<=4) j++;
                else j = 0;
                memcpy(tempdata[j],buffer,(strlen(buffer)-1)); 
            }
            
            memset(buffer, 0, sizeof(buffer));
            //send(sock, msg, strlen(msg),0);
        }
    }
}

