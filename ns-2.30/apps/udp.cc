/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (C) Xerox Corporation 1997. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Linking this file statically or dynamically with other modules is making
 * a combined work based on this file.  Thus, the terms and conditions of
 * the GNU General Public License cover the whole combination.
 *
 * In addition, as a special exception, the copyright holders of this file
 * give you permission to combine this file with free software programs or
 * libraries that are released under the GNU LGPL and with code included in
 * the standard release of ns-2 under the Apache 2.0 license or under
 * otherwise-compatible licenses with advertising requirements (or modified
 * versions of such code, with unchanged license).  You may copy and
 * distribute such a system following the terms of the GNU GPL for this
 * file and the licenses of the other code concerned, provided that you
 * include the source code of that other code when and as the GNU GPL
 * requires distribution of source code.
 *
 * Note that people who make modified versions of this file are not
 * obligated to grant this special exception for their modified versions;
 * it is their choice whether to do so.  The GNU General Public License
 * gives permission to release a modified version without this exception;
 * this exception also makes it possible to release a modified version
 * which carries forward this exception.
 */

#ifndef lint
static const char rcsid[] =
    "@(#) $Header: /tianji.cvsroot/newripple/ns-2.30/apps/udp.cc,v 1.2 2009/04/10 10:40:38 ripple Exp $ (Xerox)";
#endif

#include "udp.h"
#include "rtp.h"
#include "random.h"
#include "address.h"
#include "ip.h"


static class UdpAgentClass : public TclClass {
public:
	UdpAgentClass() : TclClass("Agent/UDP") {}
	TclObject* create(int, const char*const*) {
		return (new UdpAgent());
	}
} class_udp_agent;

UdpAgent::UdpAgent() : Agent(PT_UDP), seqno_(-1)
{
	bind("packetSize_", &size_);
}

UdpAgent::UdpAgent(packet_t type) : Agent(type)
{
	bind("packetSize_", &size_);
}

// put in timestamp and sequence number, even though UDP doesn't usually
// have one.
void UdpAgent::sendmsg(int nbytes, AppData* data, const char* flags)
{
	Packet *p;
	int n;

	assert (size_ > 0);

	n = nbytes / size_;

	if (nbytes == -1) {
		printf("Error:  sendmsg() for UDP should not be -1\n");
		return;
	}

	// If they are sending data, then it must fit within a single packet.
	if (data && nbytes > size_) {
		printf("Error: data greater than maximum UDP packet size\n");
		return;
	}

	double local_time = Scheduler::instance().clock();
	while (n-- > 0) {
		printf("udp.cc, 1111111111\n");
		p = allocpkt();
		hdr_cmn::access(p)->size() = size_;
		hdr_rtp* rh = hdr_rtp::access(p);
		rh->flags() = 0;
		rh->seqno() = ++seqno_;
		hdr_cmn::access(p)->timestamp() =
		    (u_int32_t)(SAMPLERATE*local_time);
		// add "beginning of talkspurt" labels (tcl/ex/test-rcvr.tcl)
		if (flags && (0 ==strcmp(flags, "NEW_BURST")))
			rh->flags() |= RTP_M;
		p->setdata(data);
		target_->recv(p);
	}
	n = nbytes % size_;
	if (n > 0) {
//		printf("udp.cc, 222222222\n");
		p = allocpkt();
		hdr_cmn::access(p)->size() = n;
		hdr_rtp* rh = hdr_rtp::access(p);
		rh->flags() = 0;
		rh->seqno() = ++seqno_;
		hdr_cmn::access(p)->timestamp() =
		    (u_int32_t)(SAMPLERATE*local_time);
		// add "beginning of talkspurt" labels (tcl/ex/test-rcvr.tcl)
		if (flags && (0 == strcmp(flags, "NEW_BURST")))
			rh->flags() |= RTP_M;
		p->setdata(data);


#ifdef TIANJI_RIPPLE
        struct hdr_cmn  * ch      = HDR_CMN(p);
        ch->f_list[0]        = forwarder1  ;
        ch->f_list[1]        = forwarder2  ;
        ch->f_list[2]        = forwarder3  ;
        ch->f_list[3]        = forwarder4  ;
        ch->f_list[4]        = forwarder5  ;
        ch->f_list[5]        = forwarder6  ;

        ch->afr_f_list[0]        = forwarder1  ;
        ch->afr_f_list[1]        = forwarder2  ;
        ch->afr_f_list[2]        = forwarder3  ;
        ch->afr_f_list[3]        = forwarder4  ;
        ch->afr_f_list[4]        = forwarder5  ;
        ch->afr_f_list[5]        = forwarder6  ;

        ch->mac_type         = mac_type;
#endif


		target_->recv(p);
	}
	idle();
}
void UdpAgent::recv(Packet* pkt, Handler*)
{
	if (app_ ) {
		// If an application is attached, pass the data to the app
		hdr_cmn* h = hdr_cmn::access(pkt);
		app_->process_data(h->size(), pkt->userdata());
	} else if (pkt->userdata() && pkt->userdata()->type() == PACKET_DATA) {
		// otherwise if it's just PacketData, pass it to Tcl
		//
		// Note that a Tcl procedure Agent/Udp recv {from data}
		// needs to be defined.  For example,
		//
		// Agent/Udp instproc recv {from data} {puts data}

		PacketData* data = (PacketData*)pkt->userdata();

		hdr_ip* iph = hdr_ip::access(pkt);
                Tcl& tcl = Tcl::instance();
		tcl.evalf("%s process_data %d {%s}", name(),
		          iph->src_.addr_ >> Address::instance().NodeShift_[1],
			  data->data());
	}
	Packet::free(pkt);
}


int UdpAgent::command(int argc, const char*const* argv)
{
	if (argc == 4) {
		if (strcmp(argv[1], "send") == 0) {
			PacketData* data = new PacketData(1 + strlen(argv[3]));
			strcpy((char*)data->data(), argv[3]);
			sendmsg(atoi(argv[2]), data);
			return (TCL_OK);
		}
	} else if (argc == 5) {
		if (strcmp(argv[1], "sendmsg") == 0) {
			PacketData* data = new PacketData(1 + strlen(argv[3]));
			strcpy((char*)data->data(), argv[3]);
			sendmsg(atoi(argv[2]), data, argv[4]);
			return (TCL_OK);
		}
	}



	#ifdef TIANJI_RIPPLE
	    if ( argc == 3 ) {
	        if(strcmp(argv[1], "MacType") == 0){
	        	mac_type = atoi(argv[2]);
	        	return(TCL_OK);
	        }
	    }

	    if (argc > 8 || argc < 4) {
	        if(strcmp(argv[1], "ForwardList") == 0){
	          printf("Support from 2-5 hops, I don't like %d hops!\n", argc-3);
	          exit(TCL_OK);
	        }
	    }
	    if (argc == 8) {
	        if(strcmp(argv[1], "ForwardList") == 0){
	             forwarder1=atoi(argv[2]);
	             forwarder2=atoi(argv[3]);
	             forwarder3=atoi(argv[4]);
	             forwarder4=atoi(argv[5]);
	             forwarder5=atoi(argv[6]);
	             forwarder6=atoi(argv[7]);
	        }
	        return (TCL_OK);
	    }
	    if (argc == 7) {
	        if(strcmp(argv[1], "ForwardList") == 0){
	             forwarder1=atoi(argv[2]);
	             forwarder2=atoi(argv[3]);
	             forwarder3=atoi(argv[4]);
	             forwarder4=atoi(argv[5]);
	             forwarder5=atoi(argv[6]);
	             forwarder6=-1;
	        }
	        return (TCL_OK);
	    }
	    if (argc == 6) {
	        if(strcmp(argv[1], "ForwardList") == 0){
	             forwarder1=atoi(argv[2]);
	             forwarder2=atoi(argv[3]);
	             forwarder3=atoi(argv[4]);
	             forwarder4=atoi(argv[5]);
	             forwarder5=-1;
	             forwarder6=-1;
	        }
	        return (TCL_OK);
	    }
	    if (argc == 5) {
	        if(strcmp(argv[1], "ForwardList") == 0){
	             forwarder1=atoi(argv[2]);
	             forwarder2=atoi(argv[3]);
	             forwarder3=atoi(argv[4]);
	             forwarder4=-1;
	             forwarder5=-1;
	             forwarder6=-1;
	        }
	        return (TCL_OK);
	    }
	#endif

	return (Agent::command(argc, argv));
}
