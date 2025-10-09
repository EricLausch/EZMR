#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//
#define ENTRYLEN 128

void chat (int sock, char *client_ip){

    int j = 0, z = 0, index = 0;

    char* msg ="Hello from server\n";
    char* gb = "GOODBYE CLIENT!\n";
    char entry[1024] = {0}; 
    char buffer[1024] = {0};
    char logbuffer[1024] = {0};

    char data[5][ENTRYLEN] = {"Banane", "Gulash", "Bier"};
    char tempdata[5][ENTRYLEN] = {0};
    char log[1024][ENTRYLEN] = {0};
    char newValue[ENTRYLEN] = {0};

    while(1){
        read(sock, buffer, 1024-1);

        time_t now;
	    now = time(0);
        strcpy(entry,ctime(&now));
        entry[strlen(entry)-1]='\0';

        strcat(entry," ");
        strcat(entry, client_ip);
        strcat(entry," ");
        strcat(entry,"to SERVER ");
        strcat(entry,buffer);

        if (z <= 1023){
                strncpy(log[z], entry, ENTRYLEN - 1);
                log[z][ENTRYLEN - 1] = '\0';
            z++;
        }
        else{
            z = 0;
            memset(log, 0, sizeof log);
        }

        if (strncmp(buffer, "EXIT", 4) == 0){
            send(sock, gb, strlen(gb),0);
            exit(0);
        }

        else if (strncmp(buffer, "GETLIST", 7) == 0){
            for (int i = 0; i <= (sizeof(data)/ENTRYLEN); i++)
            {
                if (data[i][0] != '\0'){
                    send(sock, data[i], strlen(data[i]),0);
                    send(sock, "\n", strlen("\n"),0);
                    memset(buffer, 0, sizeof (buffer));
                }
            }        
        }

        else if (strncmp(buffer, "GETITEM", 7) == 0){
            sscanf(buffer, "GETITEM %d", &index);
            if (index >= 0 && index < 5 && data[index][0] != '\0'){
                send(sock, data[index], strlen(data[index]), 0);
                send(sock, "\n", 1, 0);
                memset(buffer, 0, sizeof (buffer));
            } else{
                char *msg = "invalid index \n";
                send(sock, msg, strlen(msg), 0);
                memset(buffer, 0, sizeof (buffer));
            }
        }

        else if (strncmp(buffer, "SETITEM", 7) == 0){
            sscanf(buffer, "SETITEM %d %127s", &index, &newValue);
            if (index >= 0 && index < 5){
                strncpy(data[index], newValue, ENTRYLEN-1);
                data[index][ENTRYLEN - 1] = '\0';

                char *msg = "entry updated\n";
                send(sock, msg, strlen(msg),0);
            }else{
                char *msg = "invalid index\n";
                send(sock, msg, strlen(msg), 0);
            }
        }

        else if (strncmp(buffer, "GETLOG", 6) == 0){
            for (int i = 0; i <= (sizeof(log)/ENTRYLEN); i++)
            {
                if (log[i][0] != '\0'){
                    send(sock, log[i], strlen(log[i]),0);
                    memset(buffer, 0, sizeof (buffer));
                }
            }  
        }

        //send(sock, msg, strlen(msg),0);
        memset(buffer, 0, sizeof (buffer));
    }
}

int main (void){

    struct sockaddr_in server, client;
    char *client_ip;
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
        client_ip = inet_ntoa(client.sin_addr);

        if(new_sock < 0) printf("Error on accept\n");

        pid = fork();
        if (pid == 0){
            close(server_socket);
            chat(new_sock, client_ip);
            exit(0);
        }
    }
}



