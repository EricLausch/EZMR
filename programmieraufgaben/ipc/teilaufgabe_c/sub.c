#include <stdio.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <errno.h>

// structure for message queue
struct mesg_buffer {
    long mesg_type;
    char mesg_text[100];
} message;

struct msqid_ds buf;

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
        msgsnd(msgid, &message, sizeof(message.mesg_text), 0);

        ssize_t r = msgrcv(msgid, &message, sizeof(message.mesg_text), 2, 0);
        if (r == -1) {
            if (errno != EINTR) perror("msgrcv");
            continue;
        }
        printf("%s", message.mesg_text);
        if (strncmp(message.mesg_text, "BYE", 3) == 0) break;

        while ((r = msgrcv(msgid, &message, sizeof(message.mesg_text), 2, IPC_NOWAIT)) != -1) {
            printf("%s", message.mesg_text);
        }
        if (r == -1 && errno != ENOMSG) perror("msgrcv");

    }
    
    return 0;
}