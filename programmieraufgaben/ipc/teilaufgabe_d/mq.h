#define KEY 1234L
#define PERM 0666
#define MAX_MESGS 100
#define MAX_LEN 100

// structure for message queue
typedef struct {
    long mesg_type;
    char mesg_text[MAX_LEN];
} client2server;

typedef struct {
    long mesg_type;
    char mesg_text[MAX_LEN];
} server2client;

struct client_lst{
    int id;
    struct client_lst *next;
};