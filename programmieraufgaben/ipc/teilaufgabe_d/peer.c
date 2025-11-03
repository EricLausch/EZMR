#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

#define MAX_MSG_LEN 100
#define MAX_ITEMS 50

#define SEND 1
#define PING 2
#define PONG 3
#define REQ_LIST 4
#define SEND_LIST 5

struct message {
    long msg_type;
    int sender_key;
    char msg_text[MAX_MSG_LEN];
};

int qid;
key_t key;

char *list[MAX_ITEMS];
int list_size = 0;

void cleanup(int sig){
    for (int i = 0; i < list_size; i++) free(list[i]);
    if(msgctl(qid, IPC_RMID, NULL) == -1){
        perror("msgctl (IPC_RMIF)");
    }
    exit(0);
}

void update_list(int index, const char *item){
    if (index < 1 || index > list_size) {
        printf("[WARN] invalid index: %d\n", index);
        return;
    }
    free(list[index - 1]);
    list[index - 1] = strdup(item);
    printf("[INFO] item %d updated: %s\n", index, item);
}

void add_item(const char *item) {
    if (list_size < MAX_ITEMS) {
        list[list_size++] = strdup(item);
        printf("[INFO] item added: %s\n", item);
    } else {
        printf("[WARN] list full\n");
    }
}

void send_list(int target_key){
    int target_qid = msgget(target_key, 0666);
    if (target_qid == -1) return;

    struct message msg;
    msg.msg_type = SEND_LIST;
    msg.sender_key = key;
    msg.msg_text[0] = '\0';

    for (int i = 0; i < list_size; i++){
        strncat(msg.msg_text, list[i], MAX_MSG_LEN - strlen(msg.msg_text) -1);
        strncat(msg.msg_text, "\n", MAX_MSG_LEN - strlen(msg.msg_text) - 1);
    }

    msgsnd(target_qid, &msg, sizeof(msg) - sizeof(long), 0);
}

void show_list(){
    printf("---- LIST ----\n");
    for (int i = 0; i < list_size; i++){
        printf("%2d: %s\n", i + 1, list[i]);
    }
    printf("----END OF LIST----\n");
}

void send_to_peer(int target_key, long msg_type, const char *text){
    int target_qid = msgget(target_key, 0666);
    if (target_qid == -1){
        perror("mssget target");
        return;
    }

    struct message msg;
    msg.msg_type = msg_type;
    msg.sender_key = key;

    if (text) strncpy(msg.msg_text, text, MAX_MSG_LEN);
    else msg.msg_text[0] = '\0';

    msg.msg_text[MAX_MSG_LEN - 1] = '\0';

    if (msgsnd(target_qid , &msg, sizeof(msg) - sizeof(long), 0) == -1) perror("msgsnd");        
}

void *receive (void *arg){
    struct message msg;
    while (1){
        msgrcv(qid, &msg, sizeof(msg) - sizeof(long), 0, 0);

        switch (msg.msg_type) {
            case REQ_LIST:
                send_list(msg.sender_key);
                printf("[INFO] send list to %d\n", msg.sender_key);
                break;

            case SEND_LIST:
                for (int i = 0; i < list_size; i++) free(list[i]);
                list_size = 0;
                {
                    char *line = strtok(msg.msg_text, "\n");
                    while (line && list_size < MAX_ITEMS) {
                        list[list_size++] = strdup(line);
                        line = strtok(NULL, "\n");
                    }
                }
                printf("[INFO] received list from %d and saved it\n", msg.sender_key);
                break;

            case PING:
                int target_qid = msgget(msg.sender_key, 0666);
                if (target_qid != -1){
                    struct message reply;
                    reply.msg_type = PONG;
                    reply.sender_key = key;
                    snprintf(reply.msg_text, MAX_MSG_LEN, "PONG\n");
                    printf("PING\n");
                    msgsnd(target_qid, &reply, sizeof(reply) - sizeof(long), 0);
                }
                break;
            
            case PONG:
                printf("%s", msg.msg_text);
                break;

            case SEND:
                if (strncmp(msg.msg_text, "ADD ", 4) == 0) {
                    add_item(msg.msg_text + 4);
                    printf("[REMOTE ADD] %s\n", msg.msg_text + 4);
                } else if (strncmp(msg.msg_text, "UPDATE ", 7) == 0) {
                    char *tok = strtok(msg.msg_text + 7, " ");
                    if (!tok) break;
                    int index = atoi(tok);
                    char *new_value = strtok(NULL, "");
                    if (!new_value) break;
                    update_list(index, new_value);
                    printf("[REMOTE UPDATE] Index %d => %s\n", index, new_value);
                } else {
                    printf("%s", msg.msg_text);
                }
                break;
            default:
                printf("error: unknown msg_type %ld\n", msg.msg_type);

            }
            fflush(stdout);
    }
    return NULL;
}

int main(int argc, char *argv[]){
    if (argc != 2){
        printf("Usage: %s <key> key has to be a positve integer\n Example: %s 1234\n", argv[0], argv[0]);
        exit(1);
    }

    key = atoi(argv[1]);
    if (key <= 0){
        exit(1);
    }

    qid = msgget(key, IPC_CREAT | 0666);

    printf("[INFO] created msg-queue %d\n",qid);
    printf("[INFO] exit with STRG+C\n");

    signal(SIGINT, cleanup);

    pthread_t tid;
    if (pthread_create (&tid, NULL, receive, NULL) != 0){
        perror("pthread_create");
        cleanup(0);
    }
    pthread_detach(tid);

    while (1){
        char input[100];
        fflush(stdout);

        if (!fgets(input, sizeof(input), stdin)) break;
        input[strcspn(input, "\n")] = '\0';

        if (strlen(input) == 0) continue;

        if (strncmp(input, "ADD ", 4) == 0) {
            add_item(input + 4);

        } else if (strncmp(input, "UPDATE ", 7) == 0) {
            char *tok = strtok(input + 7, " ");
            if (!tok) { printf("error: missing index\n"); continue; }
            int index = atoi(tok);
            char *new_value = strtok(NULL, "");
            if (!new_value) { printf("error: missing item\n"); continue; }
            update_list(index, new_value);

        } else if (strncmp(input, "SHOW", 4) == 0) {
            show_list();

        } else if (strncmp(input, "GETLIST ", 8) == 0) {
            int target_key = atoi(input + 8);
            send_to_peer(target_key, REQ_LIST, NULL);

        } else if (strncmp(input, "SEND ", 5) == 0) {
            char *tok = strtok(input + 5, " ");
            if (!tok) { printf("error: target key missing\n"); continue; }
            int target_key = atoi(tok);

            char *cmd = strtok(NULL, "");
            if (!cmd) { printf("error: message missing\n"); continue; }
            send_to_peer(target_key, SEND, cmd);

        } else if (strncmp(input, "PING ", 5) == 0) {
            int target_key = atoi(input + 5);
            send_to_peer(target_key, PING, NULL);

        } else {
            printf("Unknown command. Use ADD, UPDATE, GETLIST, SHOW, SEND, PING\n");
        }
    }
    cleanup(0);
    return 0;
}