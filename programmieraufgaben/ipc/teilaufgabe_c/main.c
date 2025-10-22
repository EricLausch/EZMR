#include <stdio.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

#include <time.h>

#define MAX_MESGS 10
#define MAX_LEN 100

char msg_log[MAX_MESGS][MAX_LEN];
int log_count = 0;

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

int main()
{
    key_t key;
    int msgid;

    // ftok to generate unique key
    key = ftok("progfile", 65);

    // msgget creates a message queue
    // and returns identifier
    msgid = msgget(key, 0666 | IPC_CREAT);

    while (1)
    {
        // msgrcv to receive message
        if (msgrcv(msgid, &message, sizeof(message), 1, 0) == -1){
            perror("msgrcv");
            break;
        }
        // display the message
        server_log(msg_log, message.mesg_text,&log_count);
        printf("Data Received is : %s", message.mesg_text);

        if (strncmp(message.mesg_text, "PING", 4) == 0){
            message.mesg_type = 2;
            strncpy(message.mesg_text, "PONG\n", sizeof(message.mesg_text)-1);
            msgsnd(msgid, &message, sizeof(message), 0);
        }
        else if (strncmp(message.mesg_text, "SHUTDOWN", 8) == 0){
            message.mesg_type = 2;
            strncpy(message.mesg_text, "BYE\n", sizeof(message.mesg_text)-1);
            msgsnd(msgid, &message, sizeof(message), 0);
            break;
        }
        else if (strncmp(message.mesg_text, "GETLOG", 6) == 0){
            server_log(msg_log,"SENT LOG\n", &log_count);
            for (int i = 0; i < MAX_MESGS; i++){
                message.mesg_type = 2;
                strncpy(message.mesg_text, msg_log[i], sizeof(message.mesg_text)-1);
                msgsnd(msgid, &message, sizeof(message),0);
            }
            message.mesg_type = 2;
            strncpy(message.mesg_text, "------END OF LOG------\n", sizeof(message.mesg_text)-1);
            msgsnd(msgid, &message, sizeof(message), 0); 
        }
        else {
            strncpy(message.mesg_text, " \n", sizeof(message.mesg_text)-1);
            msgsnd(msgid, &message, sizeof(message), 0);
        }
    }
    
    // to destroy the message queue
    msgctl(msgid, IPC_RMID, NULL);
    return 0;
}