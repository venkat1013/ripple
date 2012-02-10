#ifndef mudpsink_h
#define mudpsink_h

#include <stdio.h>
#include "agent.h"

class mUdpSink : public Agent
{
public:
 		mUdpSink() : Agent(PT_UDP), pkt_received(0) {} 		
        	void recv(Packet*, Handler*);
        	int command(int argc, const char*const* argv);
        	void print_status();
protected:
		char tbuf[100];
		FILE *tFile;
		unsigned long int  pkt_received;
};

#endif
