#include <stdio.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

// structure for message queue
struct mesg_buffer {
    long mesg_type;
    char mesg_text[100];
} message;

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
        printf("Data Received is : %s", message.mesg_text);

        if (strncmp(message.mesg_text, "PING", 4) == 0){
            strncpy(message.mesg_text, "PONG\n", sizeof(message.mesg_text)-1);
            msgsnd(msgid, &message, sizeof(message), 0);
        }
        else if (strncmp(message.mesg_text, "SHUTDOWN", 8) == 0){
            strncpy(message.mesg_text, "BYE\n", sizeof(message.mesg_text)-1);
            msgsnd(msgid, &message, sizeof(message), 0);
            break;
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