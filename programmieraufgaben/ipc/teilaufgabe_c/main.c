#include <stdio.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

#include <time.h>

#define MAX_MESGS 100
#define MAX_LEN 100

char msg_log[MAX_MESGS][MAX_LEN];
int log_count = 0;

int msgid;

// structure for message queue
struct mesg_buffer {
    long mesg_type;
    char mesg_text[MAX_LEN];
} message;

void server_log(char log[][MAX_LEN], const char *content, int *log_count){

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

void send(const char *text, int message_type){
    message.mesg_type = message_type;
    strncpy(message.mesg_text, text, sizeof(message.mesg_text)-1);
    msgsnd(msgid, &message, sizeof(message.mesg_text), 0);
}

int main()
{
    key_t key;

    // ftok to generate unique key
    key = ftok("progfile", 65);

    // msgget creates a message queue
    // and returns identifier
    msgid = msgget(key, 0666 | IPC_CREAT);

    while (1)
    {
        // msgrcv to receive message
        if (msgrcv(msgid, &message, sizeof(message.mesg_text), 1, 0) == -1){
            perror("msgrcv");
            break;
        }
        // display the message
        server_log(msg_log, message.mesg_text,&log_count);
        printf("Data Received: %s", message.mesg_text);

        if (strncmp(message.mesg_text, "PING", 4) == 0){
            server_log(msg_log,"PONG\n", &log_count);
            send("PONG\n",2);
        }
        else if (strncmp(message.mesg_text, "SHUTDOWN", 8) == 0){
            server_log(msg_log,"BYE!\n", &log_count);
            send("BYE!\n",2);
            break;
        }
        else if (strncmp(message.mesg_text, "GETLOG", 6) == 0){
            server_log(msg_log,"SENT LOG\n", &log_count);
            for (int i = 0; i < log_count; i++){
                send(msg_log[i],2);
            }
            send("------END OF LOG------\n",2);
        }
        else if (strncmp(message.mesg_text, "EDITLOG", 7) ==0){
            int id;
            char newtxt[100];
            if (sscanf(message.mesg_text, "EDITLOG %d %s", &id, newtxt) >= 2){
                strcat(newtxt, "\n");
                server_log(msg_log, newtxt, &id);
                send("OK\n",2);
            }
        }
        else send("\n",2);

    }
    
    // to destroy the message queue
    msgctl(msgid, IPC_RMID, NULL);
    return 0;
}