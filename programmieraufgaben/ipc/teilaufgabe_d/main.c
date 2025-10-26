#include <stdio.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

#include <time.h>

#include "mq.h"

char msg_log[MAX_MESGS][MAX_LEN];
int log_count = 0;

int msgid;

struct client_message cmsg;
struct server_message smsg;

// list of connected clients, max 100 allowed
struct client_list cl[100];
int client_cnt = 0;

void server_log(char log[][MAX_LEN], const char *content, int *log_count)
{

    if (*log_count >= MAX_MESGS)
        return;

    char entry[MAX_LEN];
    time_t now = time(0);
    strcpy(entry, ctime(&now));
    entry[strlen(entry) - 1] = '\0';

    strcat(entry, " ");
    strcat(entry, content);

    strncpy(log[*log_count], entry, MAX_LEN - 1);
    log[*log_count][MAX_LEN - 1] = '\0';
    (*log_count)++;
}

void send(const char *text, int client_qid)
{
    struct server_message smsg;

    smsg.mesg_type = 2;
    strncpy(smsg.mesg_text, text, sizeof(smsg.mesg_text) - 1);
    msgsnd(client_qid, &smsg, sizeof(smsg.mesg_text), 0);
}

int find_qid_by_name(const char *name) {
    for (int i = 0; i < client_cnt; i++) {
        if (strcmp(cl[i].username, name) == 0)
            return cl[i].qid;
    }
    return -1;
}

int main()
{
    int server_id = msgget(SERVER_KEY, PERM | IPC_CREAT);

    while (1)
    {
        // msgrcv to receive message
        if (msgrcv(server_id, &cmsg, sizeof(cmsg) - sizeof(long), 1, 0) == -1)
        {
            perror("msgrcv");
            break;
        }
        // display the message
        server_log(msg_log, cmsg.mesg_text, &log_count);
        printf("Data Received (%d): %s", cmsg.client_qid, cmsg.mesg_text);

        // add a new Client to the client list and
        if (strncmp(cmsg.mesg_text, "CONNECT", 7) == 0)
        {
            sscanf(cmsg.mesg_text, "CONNECT %s", cl[client_cnt].username);
            cl[client_cnt].qid = cmsg.client_qid;
            client_cnt++;
            printf("[Server] Hello: %s (%d)\n", cl[client_cnt - 1].username, cmsg.client_qid);
            send("Hello Client!", cmsg.client_qid);
        }

        else if (strncmp(cmsg.mesg_text, "SEND", 4) == 0){
            int target;
            target = find_qid_by_name(cmsg.username);
            printf("%d", target);
            send(cmsg.mesg_text, target);
        }

        else if (strncmp(cmsg.mesg_text, "PING", 4) == 0)
        {
            server_log(msg_log, "PONG", &log_count);
            send("PONG\n", cmsg.client_qid);
        }

        else if (strncmp(cmsg.mesg_text, "SHUTDOWN", 8) == 0)
        {
            server_log(msg_log, "BYE!", &log_count);
            send("BYE!\n", cmsg.client_qid);
            break;
        }

        else if (strncmp(cmsg.mesg_text, "GETLOG", 6) == 0)
        {
            server_log(msg_log, "SENT LOG\n", &log_count);
            for (int i = 0; i < log_count; i++)
            {
                send(msg_log[i], cmsg.client_qid);
            }
            send("------END OF LOG------\n", cmsg.client_qid);
        }

        else if (strncmp(cmsg.mesg_text, "EDITLOG", 7) == 0)
        {
            int id;
            char newtxt[100];
            if (sscanf(cmsg.mesg_text, "EDITLOG %d %s", &id, newtxt) >= 2)
            {
                strcat(newtxt, "\n");
                server_log(msg_log, newtxt, &id);
                send("OK", cmsg.client_qid);
            }
        }
    }
    // to destroy the message queue
    msgctl(msgid, IPC_RMID, NULL);
    return 0;
}