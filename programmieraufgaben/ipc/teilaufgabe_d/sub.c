#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include "mq.h"

int client_qid;
struct client_message cmsg;

void cleanup(int sig){
    if(msgctl(client_qid, IPC_RMID, NULL) == -1){
        perror("msgctl (IPC_RMIF)");
    }
    exit(0);
}

void send(){

    int receiver_id;
    char input[100];

    printf("> ");
    fgets(input,sizeof(input),stdin);
    sscanf(input, "%d", &receiver_id);
    char *msg = strchr(input, ' ');
    strcpy(cmsg.mesg_text, msg ? msg + 1 : "");

    if (msgsnd(receiver_id, &cmsg, sizeof(cmsg.mesg_text), 0) == -1){
        perror("msgsnd");
    }
}

int main()
{
    struct server_message smsg;
    
    int server_id = msgget(SERVER_KEY, 0);
    client_qid = msgget(IPC_PRIVATE, PERM | IPC_CREAT);

    cmsg.mesg_type = 1;
    cmsg.client_qid = client_qid;

    signal(SIGINT, cleanup);

    while (1)
    {


        // msgsnd to send message
        send();

        ssize_t r = msgrcv(client_qid, &smsg, sizeof(smsg.mesg_text), 2, 0);
        if (r == -1) {
            if (errno != EINTR) perror("msgrcv");
            continue;
        }
        printf("%s", smsg.mesg_text);
        if (strncmp(smsg.mesg_text, "BYE", 3) == 0) break;

        while ((r = msgrcv(client_qid, &smsg, sizeof(smsg.mesg_text), 2, IPC_NOWAIT)) != -1) {
            printf("%s", smsg.mesg_text);
        }
        if (r == -1 && errno != ENOMSG) perror("msgrcv");

    }
    
    return 0;
}