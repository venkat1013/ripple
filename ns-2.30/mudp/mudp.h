#ifndef ns_mudp_h
#define ns_mudp_h

#include "udp.h"

class mUdpAgent : public UdpAgent {
public:
	mUdpAgent();
	virtual void sendmsg(int nbytes, AppData* data, const char *flags = 0);
	virtual int command(int argc, const char*const* argv);


#ifdef TIANJI_RIPPLE
	int mac_type;
    int forwarder1;
    int forwarder2;
    int forwarder3;
    int forwarder4;
    int forwarder5;
    int forwarder6;
    int forwarder7;
    int forwarder8;
#endif


protected:
	int id_;
	char BWfile[100];
	FILE *BWFile;
	int openfile;
};

#endif
