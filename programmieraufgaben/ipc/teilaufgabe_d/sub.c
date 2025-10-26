#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/select.h>
#include <unistd.h>
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

void send(int server_qid){

    fgets(cmsg.mesg_text,sizeof(cmsg.mesg_text),stdin);

    if (strncmp(cmsg.mesg_text, "SEND",4) == 0){

        char target [32];
        char msg[100];
        if (sscanf(cmsg.mesg_text, "SEND %31s %[^\n]", target, msg) == 2){
            strncpy(cmsg.username, target, sizeof(cmsg.username) - 1);
            strncpy(cmsg.mesg_text, msg, sizeof(cmsg.mesg_text) - 1);
        }
    }
    else cmsg.target_qid = server_qid;

    if (msgsnd(server_qid, &cmsg, sizeof(cmsg) - sizeof(long), 0) == -1){
        perror("msgsnd");
    }
}

int main()
{
    struct server_message smsg;
    
    int server_qid = msgget(SERVER_KEY, 0);
    client_qid = msgget(IPC_PRIVATE, PERM | IPC_CREAT);

    cmsg.mesg_type = 1;
    cmsg.client_qid = client_qid;

    signal(SIGINT, cleanup);

    while (1)
    {

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;

        int ret = select(STDIN_FILENO +1, &readfds, NULL, NULL, &tv);

        // msgsnd to send message
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) send(server_qid);

        while (msgrcv(client_qid, &smsg, sizeof(smsg.mesg_text), 0, IPC_NOWAIT) != -1){
            printf("%s\n", smsg.mesg_text);
            fflush(stdout);
        }
        if (errno != ENOMSG) perror("msgrcv");
    }
    
    return 0;
}
