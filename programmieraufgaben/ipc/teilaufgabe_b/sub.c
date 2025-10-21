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
    msgid = msgget(key, 0666);

    while (1)
    {
        printf("> ");
        fgets(message.mesg_text,sizeof(message.mesg_text),stdin);

        // msgsnd to send message
        message.mesg_type = 1;
        msgsnd(msgid, &message, sizeof(message), 0);

        // receive answer of msg_type 2
        msgrcv(msgid, &message, sizeof(message), 1, 0);
        printf("answer from MAIN: %s", message.mesg_text);
        if (strncmp(message.mesg_text, "BYE", 3) == 0) break;
    }
    
    return 0;
}