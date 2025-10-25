#define MAX_LEN 100
#define MAX_MESGS 100
#define USERNAME_LEN 32

#define SERVER_KEY 1234L
#define PERM 0666

struct client_message{
    long mesg_type;
    int client_qid;
    int target_qid;
    char username[USERNAME_LEN];
    char mesg_text[MAX_LEN];
};

struct server_message {
    long mesg_type;
    char mesg_text[MAX_LEN];
};

struct client_list{
    int qid;
    char username[USERNAME_LEN];
};
