#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#define PORT 12345
#define MAX_MSGS 100
#define MAX_LEN 1024

char msg_log[MAX_MSGS][MAX_LEN];

void chat(int, const char *);

int main (void){

    struct sockaddr_in server, client;
    int new_sock, pid;
    socklen_t len;
    char *client_ip;

    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) printf("Error creating the socket\n");

    memset( &server, 0, sizeof (server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    server.sin_port = htons(PORT);

    if(bind(server_socket, (struct sockaddr*)&server, sizeof(server)) < 0){
        perror("Binding error");
        exit(EXIT_FAILURE);
    }

    listen(server_socket, 3);
    len = sizeof(client);

    printf("Server startet on port %d\n", PORT);

    while (1) {

        new_sock = accept(server_socket,(struct sockaddr*)&client, &len);
        client_ip = inet_ntoa(client.sin_addr);
        
        if(new_sock < 0){
            perror("Error on accept");
            continue;
        }

        int pid = fork();
        if (pid == 0){
            close(server_socket);
            chat(new_sock, client_ip);
            close(new_sock);
            exit(0);
        }
        
        close(new_sock); 
    }
}

void chat (int sock, const char *client_ip){
    
    char* msg;
    char buffer[1024] = {0};
    char entry[1024] = {0};
    int log_count = 0;

    while(1){
        read(sock, buffer, sizeof(buffer)-1);
        printf("%s",buffer);

        time_t now;
        now = time(0);

        strcpy(entry,ctime(&now));
        entry[strlen(entry)-1]='\0';

        strcat(entry," ");
        strcat(entry, client_ip);
        strcat(entry," ");
        strcat(entry,"to SERVER ");
        strcat(entry,buffer);

        if (log_count < MAX_MSGS){
            strncpy(msg_log[log_count], entry, MAX_LEN -1);
            msg_log[log_count][MAX_LEN -1] = '\0';
            log_count++;
        }
        
        if (strncmp(buffer, "PING", 4) == 0){
            msg = "PONG\n";
            send(sock, msg, strlen(msg),0);
        }
        else if (strncmp(buffer, "GETLOG",6) == 0){
            for (int i = 0; i < sizeof(msg_log)/MAX_LEN; i++){
                if (msg_log[i][0] != '\0'){
                    send(sock, msg_log[i], strlen(msg_log[i]), 0);
                }
            }
            msg = "---END OF LOG---\n";
            send(sock, msg, strlen(msg), 0);
        }
        else send(sock, "\0", 1 ,0);
    
        memset(buffer, 0, sizeof(buffer));
    }
}

