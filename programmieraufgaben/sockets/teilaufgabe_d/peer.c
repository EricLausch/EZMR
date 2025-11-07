#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>

#define PORT 12345
#define MAX_MSGS 100
#define MAX_LEN 1024

char msg_log[MAX_MSGS][MAX_LEN];
int log_count = 0;

pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct{
    int sock;
    char peer_ip[INET_ADDRSTRLEN];
}client_info_t;

void server_log(const char *sender, const char *receiver, const char *content,
                char log[][MAX_LEN], int *log_count, int mode){

    pthread_mutex_lock(&log_mutex);
    if (*log_count >= MAX_MSGS) return;

    char entry[MAX_LEN];

    // add timestamp to entry
    time_t now;
    now = time(0);
    strcpy(entry,ctime(&now));
     entry[strlen(entry)-1]='\0';

    // add content to entry
    strcat(entry," ");
    strcat(entry, sender);
    strcat(entry," to ");
    strcat(entry,receiver);
    strcat(entry," ");
    strcat(entry,content);
    if (mode == 1) strcat(entry, " (edited)\n");

    strncpy(log[*log_count], entry, MAX_LEN -1);
    log[*log_count][MAX_LEN -1] = '\0';
    if (mode == 0) (*log_count)++;
    pthread_mutex_unlock(&log_mutex);
}

void* comm (void* arg){
    client_info_t* info = (client_info_t*) arg;
    int sock = info -> sock;
    char* peer_ip = info -> peer_ip;
    free(info);
    
    char buffer[1024] = {0};
    char* msg;

    while(1){
        int n = read(sock, buffer, sizeof(buffer)-1);
        if (n <= 0) break;
        buffer[n] = '\0';
        printf("[%s] %s", peer_ip, buffer);
        fflush(stdout);

        server_log(peer_ip, "LOCAL", buffer, msg_log, &log_count, 0);
        
        if (strncmp(buffer, "PING", 4) == 0){
            msg = "PONG\n";
            send(sock, msg, strlen(msg),0);
            server_log("LOCAL", peer_ip, msg, msg_log, &log_count, 0);
        }
        else if (strncmp(buffer, "GETLOG",6) == 0){
            server_log("LOCAL", peer_ip, "SENT LOG\n", msg_log, &log_count, 0);
            pthread_mutex_lock(&log_mutex);
            for (int i = 0; i < MAX_MSGS; i++){
                if (msg_log[i][0] != '\0'){
                    send(sock, msg_log[i], strlen(msg_log[i]), 0);
                }
            }
            pthread_mutex_unlock(&log_mutex);
            msg = "---END OF LOG---\n";
            send(sock, msg, strlen(msg), 0);
        }
        else if (strncmp(buffer, "EDITLOG",7) == 0){
            int id = 0;
            char newtxt[300] = {0};
            if (sscanf(buffer, "EDITLOG %d %[^\n]", &id, newtxt) >= 2){
                server_log(peer_ip, "LOCAL", newtxt, msg_log, &id, 1);
                send(sock, "\0", 1 ,0);
            }
        }
        else send(sock, "\0", 1 ,0);
    
        memset(buffer, 0, sizeof(buffer));
    }
    close(sock);
}

void* server_thread(void* arg){
    int server_socket = socket(AF_INET, SOCK_STREAM,0);
    if(server_socket<0){ perror("socket"); return NULL; }

    struct sockaddr_in server;
    memset(&server,0,sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    server.sin_port = htons(PORT);

    if(bind(server_socket,(struct sockaddr*)&server,sizeof(server))<0){
        perror("bind"); return NULL;
    }

    listen(server_socket,5);
    printf("server is running on port: %d\n",PORT);

    while(1){
        struct sockaddr_in client;
        socklen_t len=sizeof(client);
        int new_sock = accept(server_socket,(struct sockaddr*)&client,&len);
        if(new_sock<0) continue;

        client_info_t* info = malloc(sizeof(client_info_t));
        info->sock = new_sock;
        strcpy(info->peer_ip, inet_ntoa(client.sin_addr));

        pthread_t tid;
        pthread_create(&tid,NULL,comm,info);
        pthread_detach(tid);
    }
    close(server_socket);
    return NULL;
}

void client_comm(const char* peer_ip){
    int sock = socket(AF_INET, SOCK_STREAM,0);
    if(sock<0){ perror("Client socket"); return; }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    inet_pton(AF_INET,peer_ip,&server_addr.sin_addr);

    if(connect(sock,(struct sockaddr*)&server_addr,sizeof(server_addr))<0){
        perror("connect failed");
        return;
    }

    printf("connected with: %s Commands: PING, GETLOG, EDITLOG <id> <text>, EXIT\n",peer_ip);

    char buffer[1024];
    while(1){
        printf(">> "); fflush(stdout);
        if(fgets(buffer,sizeof(buffer),stdin)==NULL) continue;

        if(strncmp(buffer,"EXIT",4)==0){
            printf("disconnected from server. reconnect with CONNECT <IP>\n");
            break;
        }

        send(sock,buffer,strlen(buffer),0);

        int n;
        while((n=read(sock,buffer,sizeof(buffer)-1))>0){
            buffer[n]='\0';
            printf("%s",buffer);

            if(strncmp(buffer,"---END OF LOG---",15)==0 || strncmp(buffer,"PONG",4)==0) break;
        }

        if(n<=0){
            printf("connection closed.\n");
            break;
        }
    }

    close(sock);
}

void* input_thread(void* arg){
    char line[256];
    while(1){
        printf(">> "); fflush(stdout);
        if(fgets(line,sizeof(line),stdin)==NULL) continue;

        if(strncmp(line,"CONNECT",7)==0){
            char peer[64];
            if(sscanf(line,"CONNECT %s",peer)==1){
                client_comm(peer);
            } else {
                printf("Usage: CONNECT <IP>\n");
            }
        } else if(strncmp(line,"EXIT",4)==0){
            printf("closing...\n");
            exit(0);
        } else {
            printf("unknown command! CONNECT <IP> or EXIT\n");
        }
    }
    return NULL;
}

int main(){
    pthread_t server_tid, input_tid;
    pthread_create(&server_tid,NULL,server_thread,NULL);
    pthread_create(&input_tid,NULL,input_thread,NULL);

    pthread_join(server_tid,NULL);
    pthread_join(input_tid,NULL);
    return 0;
}




