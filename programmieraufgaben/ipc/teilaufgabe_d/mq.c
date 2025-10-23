#include <stdio.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "mq.h"

char msg_log[MAX_MESGS][MAX_LEN];
int log_count = 0;

client2server c2s;
server2client s2c;

static int setup_server(key_t key, int flag){
    int r;
    r = msgget(key,flag);

    return r;
}

static int setup_client(key_t key, int flag){
    int r;
    r = msgget(key, flag);

    return r;
}

void server_log(int msgid, char log[][MAX_LEN], const char *content, int *log_count){

    if (*log_count >= MAX_MESGS) return;

    char entry[MAX_LEN];
    time_t now = time(0);
    strcpy(entry,ctime(&now));
    entry[strlen(entry)-1]='\0';

    strcat(entry, " ");
    strcat(entry, content);

    strncpy(log[*log_count], entry, MAX_LEN -1);
    log[*log_count][MAX_LEN -1] = '\0';
    (*log_count)++;
}


void* server()
{
    int r, client_id;
    char buffer[MAX_LEN];

    int server_id = setup_server(KEY, PERM | IPC_CREAT);

    while(1){
       r = msgrcv(server_id, &c2s, sizeof(c2s.mesg_text), 0, 0);
       if(c2s.mesg_type == 1){
        sscanf(c2s.mesg_text, "%d", &client_id);
       }
       else{
        printf ("%s", c2s.mesg_text);
       }
    }

    return NULL;
}

void* client()
{ 
    int r;
    int client_id, server_id;

    //Q to server
    server_id = setup_client (KEY, 0);

    client_id = setup_client (IPC_PRIVATE, PERM | IPC_CREAT);

    while (1)
    {
        c2s.mesg_type = 2;
        sprintf(c2s.mesg_text, "%d: Hello\n", client_id);
        r = msgsnd(server_id, &c2s, sizeof(c2s.mesg_text), 0);
        sleep(1);
    }
    msgctl (client_id, IPC_RMID, NULL);
    return NULL;
}


int main(){
    pthread_t server_tid, client_tid;
    pthread_create(&server_tid,NULL,server,NULL);
    pthread_create(&client_tid,NULL,client,NULL);

    pthread_join(server_tid,NULL);
    pthread_join(client_tid,NULL);

    return 0;
}