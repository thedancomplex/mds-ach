/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#ifdef __cplusplus
extern "C" {
#endif

typedef int mds_can_t;

extern mds_can_t mds_socket[4];

int sendCan(mds_can_t, struct can_frame *f);
void openAllCAN(int c);
int readCan(mds_can_t skt, struct can_frame *f, double timeoD);
int flushCan(mds_can_t skt, int timeOut, double giveUp);

#ifdef __cplusplus
}
#endif

