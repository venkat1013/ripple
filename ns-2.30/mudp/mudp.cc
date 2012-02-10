#include "mudp.h"
#include "rtp.h"
#include "random.h"
#include "address.h"
#include "ip.h"


static class mUdpAgentClass : public TclClass {
public:
	mUdpAgentClass() : TclClass("Agent/mUDP") {}
	TclObject* create(int, const char*const*) {
		return (new mUdpAgent());
	}
} class_mudp_agent;

mUdpAgent::mUdpAgent() : id_(0), openfile(0)
{
	bind("packetSize_", &size_);
	UdpAgent::UdpAgent();
}

void mUdpAgent::sendmsg(int nbytes, AppData* data, const char* flags)
{
	Packet *p;
	int n;
	char buf[100]; //added by smallko

	if (size_)
		n = nbytes / size_;
	else
		printf("Error: mUDP size = 0\n");

	if (nbytes == -1) {
		printf("Error:  sendmsg() for UDP should not be -1\n");
		return;
	}

	// If they are sending data, then it must fit within a single packet.
	if (data && nbytes > size_) {
		printf("Error: data greater than maximum mUDP packet size\n");
		return;
	}

	double local_time = Scheduler::instance().clock();
	while (n-- > 0) {
		p = allocpkt();
		hdr_cmn::access(p)->size() = size_;
		hdr_rtp* rh = hdr_rtp::access(p);
		rh->flags() = 0;
		rh->seqno() = ++seqno_;
		hdr_cmn::access(p)->timestamp() =
		    (u_int32_t)(SAMPLERATE*local_time);
		hdr_cmn::access(p)->sendtime_ = local_time;	// (smallko)
		if(openfile!=0){
			hdr_cmn::access(p)->pkt_id_ = id_++;
			sprintf(buf, "%-16d %-16f\n", hdr_cmn::access(p)->pkt_id_, local_time);
			fwrite(buf, strlen(buf), 1, BWFile);
		}
		// add "beginning of talkspurt" labels (tcl/ex/test-rcvr.tcl)
		if (flags && (0 ==strcmp(flags, "NEW_BURST")))
			rh->flags() |= RTP_M;
		p->setdata(data);
		target_->recv(p);
	}
	n = nbytes % size_;
	if (n > 0) {
		p = allocpkt();
		hdr_cmn::access(p)->size() = n;
		hdr_rtp* rh = hdr_rtp::access(p);
		rh->flags() = 0;
		rh->seqno() = ++seqno_;
		hdr_cmn::access(p)->timestamp() =
		    (u_int32_t)(SAMPLERATE*local_time);
		hdr_cmn::access(p)->sendtime_ = local_time;	// (smallko)
		if(openfile!=0){
			hdr_cmn::access(p)->pkt_id_ = id_++;
//			sprintf(buf, "%-16d %-16f\n", hdr_cmn::access(p)->pkt_id_, local_time);
			sprintf(buf, "%-16d %-16f\n", hdr_cmn::access(p)->uid(), local_time);
			fwrite(buf, strlen(buf), 1, BWFile);
		}
		// add "beginning of talkspurt" labels (tcl/ex/test-rcvr.tcl)
		if (flags && (0 == strcmp(flags, "NEW_BURST")))
			rh->flags() |= RTP_M;
		p->setdata(data);


		struct hdr_cmn  * ch      = HDR_CMN(p);
        ch->f_list[0]        = forwarder1  ;
        ch->f_list[1]        = forwarder2  ;
        ch->f_list[2]        = forwarder3  ;
        ch->f_list[3]        = forwarder4  ;
        ch->f_list[4]        = forwarder5  ;
        ch->f_list[5]        = forwarder6  ;
        ch->f_list[6]        = forwarder7  ;
        ch->f_list[7]        = forwarder8  ;

        ch->afr_f_list[0]        = forwarder1  ;
        ch->afr_f_list[1]        = forwarder2  ;
        ch->afr_f_list[2]        = forwarder3  ;
        ch->afr_f_list[3]        = forwarder4  ;
        ch->afr_f_list[4]        = forwarder5  ;
        ch->afr_f_list[5]        = forwarder6  ;
        ch->afr_f_list[6]        = forwarder7  ;
        ch->afr_f_list[7]        = forwarder8  ;

        ch->mac_type         = mac_type;

        target_->recv(p);
	}
	idle();
}

int mUdpAgent::command(int argc, const char*const* argv)
{
	if(argc ==2) {		//added by smallko
		if (strcmp(argv[1], "closefile") == 0) {
			if(openfile==1)
				fclose(BWFile);
			return (TCL_OK);
		}

	}

	if (argc ==3) {  	//added by smallko
		if (strcmp(argv[1], "set_filename") == 0) {
			strcpy(BWfile, argv[2]);
			BWFile = fopen(BWfile, "w");
			openfile=1;
			return (TCL_OK);
		}
	}



    if ( argc == 3 ) {
        if(strcmp(argv[1], "MacType") == 0){
        	mac_type = atoi(argv[2]);
        	return(TCL_OK);
        }
    }

    if (argc > 10 || argc < 4) {
        if(strcmp(argv[1], "ForwardList") == 0){
          printf("Support from 2-7 hops, I don't like %d hops!\n", argc-3);
          exit(TCL_OK);
        }
    }
    if (argc == 10) {// 7hops
        if(strcmp(argv[1], "ForwardList") == 0){
             forwarder1=atoi(argv[2]);
             forwarder2=atoi(argv[3]);
             forwarder3=atoi(argv[4]);
             forwarder4=atoi(argv[5]);
             forwarder5=atoi(argv[6]);
             forwarder6=atoi(argv[7]);
             forwarder7=atoi(argv[8]);
             forwarder8=atoi(argv[9]);
        }
        return (TCL_OK);
    }
    if (argc == 9) {// 6hops
        if(strcmp(argv[1], "ForwardList") == 0){
             forwarder1=atoi(argv[2]);
             forwarder2=atoi(argv[3]);
             forwarder3=atoi(argv[4]);
             forwarder4=atoi(argv[5]);
             forwarder5=atoi(argv[6]);
             forwarder6=atoi(argv[7]);
             forwarder7=atoi(argv[8]);
             forwarder8=-1;
        }
        return (TCL_OK);
    }

    if (argc == 8) {// 5hops
        if(strcmp(argv[1], "ForwardList") == 0){
             forwarder1=atoi(argv[2]);
             forwarder2=atoi(argv[3]);
             forwarder3=atoi(argv[4]);
             forwarder4=atoi(argv[5]);
             forwarder5=atoi(argv[6]);
             forwarder6=atoi(argv[7]);
             forwarder7=-1;
             forwarder8=-1;
        }
        return (TCL_OK);
    }
    if (argc == 7) {// 4hops
        if(strcmp(argv[1], "ForwardList") == 0){
             forwarder1=atoi(argv[2]);
             forwarder2=atoi(argv[3]);
             forwarder3=atoi(argv[4]);
             forwarder4=atoi(argv[5]);
             forwarder5=atoi(argv[6]);
             forwarder6=-1;
             forwarder7=-1;
             forwarder8=-1;
        }
        return (TCL_OK);
    }
    if (argc == 6) {// 3hops
        if(strcmp(argv[1], "ForwardList") == 0){
             forwarder1=atoi(argv[2]);
             forwarder2=atoi(argv[3]);
             forwarder3=atoi(argv[4]);
             forwarder4=atoi(argv[5]);
             forwarder5=-1;
             forwarder6=-1;
             forwarder7=-1;
             forwarder8=-1;
        }
        return (TCL_OK);
    }
    if (argc == 5) { // 2hops
        if(strcmp(argv[1], "ForwardList") == 0){
             forwarder1=atoi(argv[2]);
             forwarder2=atoi(argv[3]);
             forwarder3=atoi(argv[4]);
             forwarder4=-1;
             forwarder5=-1;
             forwarder6=-1;
             forwarder7=-1;
             forwarder8=-1;
        }
        return (TCL_OK);
    }



	return (UdpAgent::command(argc, argv));
}
