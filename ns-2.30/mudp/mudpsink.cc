#include <stdio.h>
#include <stdlib.h>
#include "mudpsink.h"
#include "ip.h"
#include "udp.h"
#include "rtp.h"

static class mUdpSinkClass : public TclClass {
public:
        mUdpSinkClass() : TclClass("Agent/mUdpSink") {}
        TclObject* create(int, const char*const*) {
                return (new mUdpSink);
        }
} class_mudpsink;

void mUdpSink::recv(Packet* pkt, Handler*)
{
  	hdr_cmn* hdr=hdr_cmn::access(pkt);

	pkt_received+=1;
//	printf("%-16d %-16f %-16f %-16f\n", hdr->uid(), hdr->sendtime_, Scheduler::instance().clock(),  Scheduler::instance().clock()-hdr->sendtime_);

	// end-to-end delay > 0.125 are considered as losses!!!
	if ( Scheduler::instance().clock()-hdr->sendtime_ <= 0.125 )
	    fprintf(tFile,"%-16d %-16f %-16f %-16f\n", hdr->uid(), hdr->sendtime_, Scheduler::instance().clock(),  Scheduler::instance().clock()-hdr->sendtime_);
//	printf("%-16d %-16f %-16f %-16f\n", hdr->pkt_id_, hdr->sendtime_, Scheduler::instance().clock(),  Scheduler::instance().clock()-hdr->sendtime_);
//	fprintf(tFile,"%-16d %-16f %-16f %-16f\n", hdr->pkt_id_, hdr->sendtime_, Scheduler::instance().clock(),  Scheduler::instance().clock()-hdr->sendtime_);

  	if (app_)
               app_->recv(hdr_cmn::access(pkt)->size());

        Packet::free(pkt);
}

int mUdpSink::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();

	if (strcmp(argv[1], "set_filename") == 0) {
		strcpy(tbuf, argv[2]);
		tFile = fopen(tbuf, "w");
		return (TCL_OK);
	}

	if (strcmp(argv[1], "closefile") == 0) {
		fclose(tFile);
		return (TCL_OK);
	}

	if(strcmp(argv[1],"printstatus")==0) {
		print_status();
		return (TCL_OK);
	}

	return (Agent::command(argc, argv));
}

void mUdpSink::print_status()
{
	printf("mUdpSink2)Total packets received:%ld\n", pkt_received);
}
