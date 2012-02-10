/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 1997 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the Daedalus Research
 *	Group at the University of California Berkeley.
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Contributed by the Daedalus Research Group, http://daedalus.cs.berkeley.edu
 */

#ifndef lint
static const char rcsid[] =
    "@(#) $Header: /tianji.cvsroot/newripple/ns-2.30/mac/ll.cc,v 1.7 2009/07/06 14:50:01 ripple Exp $ (UCB)";
#endif

#include <errmodel.h>
#include <mac.h>
#include <ll.h>
#include <address.h>
#include <dsr/hdr_sr.h>

#include <packet.h>
#include <afr/mac-afr.h>
#include <mac-ripple.h>
#include <mac-exor.h>
#include <mcexor/mac-mcexor.h>

//#define TIANJI_NEW_DEBUG

int hdr_ll::offset_;

static class LLHeaderClass : public PacketHeaderClass {
public:
	LLHeaderClass()	: PacketHeaderClass("PacketHeader/LL",
					    sizeof(hdr_ll)) {
		bind_offset(&hdr_ll::offset_);
	}
} class_hdr_ll;


static class LLClass : public TclClass {
public:
	LLClass() : TclClass("LL") {}
	TclObject* create(int, const char*const*) {
		return (new LL);
	}
} class_ll;


LL::LL() : LinkDelay(), seqno_(0), ackno_(0), macDA_(0), ifq_(0),
	mac_(0), lanrouter_(0), arptable_(0), varp_(0),
	downtarget_(0), uptarget_(0)
{
	bind("macDA_", &macDA_);
#ifdef TIANJI_RIPPLE
        bind("totNumSTA_", &totNumSTA_);
#endif
}

int LL::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
	if (argc == 3) {
		if (strcmp(argv[1], "ifq") == 0) {
			ifq_ = (Queue*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if(strcmp(argv[1], "arptable") == 0) {
                        arptable_ = (ARPTable*)TclObject::lookup(argv[2]);
                        assert(arptable_);
                        return TCL_OK;
                }
		if(strcmp(argv[1], "varp") == 0) {
                        varp_ = (VARPTable*)TclObject::lookup(argv[2]);
                        assert(varp_);
                        return TCL_OK;
                }
		if (strcmp(argv[1], "mac") == 0) {
			mac_ = (Mac*) TclObject::lookup(argv[2]);
			printf("ll.cc, command, mac addr=%d\n", mac_->addr());
                        assert(mac_);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "down-target") == 0) {
			downtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "up-target") == 0) {
			uptarget_ = (NsObject*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "lanrouter") == 0) {
			lanrouter_ = (LanRouter*) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}

	}
	else if (argc == 2) {
		if (strcmp(argv[1], "ifq") == 0) {
			tcl.resultf("%s", ifq_->name());
			return (TCL_OK);
		}
		if (strcmp(argv[1], "mac") == 0) {
			tcl.resultf("%s", mac_->name());
			return (TCL_OK);
		}
		if (strcmp(argv[1], "down-target") == 0) {
			tcl.resultf("%s", downtarget_->name());
			return (TCL_OK);
		}
		if (strcmp(argv[1], "up-target") == 0) {
			tcl.resultf("%s", uptarget_->name());
			return (TCL_OK);
		}
	}
	return LinkDelay::command(argc, argv);
}



void LL::recv(Packet* p, Handler* /*h*/)
{
	hdr_cmn *ch = HDR_CMN(p);
	//char *mh = (char*) HDR_MAC(p);
	//struct hdr_sr *hsr = HDR_SR(p);

	/*
	 * Sanity Check
	 */
	assert(initialized());

	//if(p->incoming) {
	//p->incoming = 0;
	//}
	// XXXXX NOTE: use of incoming flag has been depracated; In order to track direction of pkt flow, direction_ in hdr_cmn is used instead. see packet.h for details.

	// If direction = UP, then pass it up the stack
	// Otherwise, set direction to DOWN and pass it down the stack
	if(ch->direction() == hdr_cmn::UP) {
		//if(mac_->hdr_type(mh) == ETHERTYPE_ARP)
		if(ch->ptype_ == PT_ARP)
			arptable_->arpinput(p, this);
		else
			uptarget_ ? sendUp(p) : drop(p);
		return;
	}

	ch->direction() = hdr_cmn::DOWN;
	sendDown(p);
}


void LL::sendDown(Packet* p)
{
	hdr_cmn *ch = HDR_CMN(p);
	hdr_ll *llh = HDR_LL(p);
	char *mh;

#ifdef TIANJI_RIPPLE
//	printf("ll.cc, mac_type=%d, pid=%d. \n", ch->mac_type, ch->uid());
	switch (ch->mac_type) {
		case 0: // RIPPLE
			mh = (char*)HDR_MAC_RIPPLE_DATA(p);
			break;
		case 1: // LPR
			mh = (char*)HDR_MAC(p);
			break;
		case 2: // SPR/ETX
			mh = (char*)HDR_MAC(p);
			break;
		case 3: // AFR
			mh = (char*)HDR_MAC_RIPPLE_DATA(p);
			break;
		case 4: // ExOR
//			mh = (char*)HDR_MAC_EXOR_DATA(p);
			break;
		case 5: // MCExOR
//			mh = (char*)HDR_MAC_MCEXOR_DATA(p);
			break;
	}
#endif

	llh->seqno_ = ++seqno_;
	llh->lltype() = LL_DATA;

	mac_->hdr_src(mh, mac_->addr());
	mac_->hdr_type(mh, ETHERTYPE_IP);
	int tx = 0;
	int i;

#ifdef TIANJI_RIPPLE
	if ( ch->mac_type == 0 ) // RIPPLE
	{
        for ( i=MAX_FORWARDERS-1; i>=0; i-- ) {
            if ( ch->f_list[i] >= 0 ) {
				// assign mac dst for each packet
				 mac_->hdr_dst((char*) HDR_MAC_RIPPLE_DATA(p), ch->f_list[i]);
				 break;
            }
        }
    }
    if ( ch->mac_type == 1 ) // LPR
	{
        for ( i=MAX_FORWARDERS-1; i>=0; i-- ) {
            if ( ch->f_list[i] >= 0 ) {
				// assign mac dst for each packet
				 mac_->hdr_dst((char*) HDR_MAC(p), ch->f_list[i]);
				 break;
            }
        }
//     	printf("ll.cc, type=%d, mac dst=%d, addr=%d\n", ch->mac_type, ch->f_list[i], mac_->addr());

    }
    if ( ch->mac_type == 2 ) // SPR and ETX
	{
    	if ( MAX_FORWARDERS == 2 ) {
    		printf("MAX_FORWARDERS == 2 is not enough for SPR!\n");
    		exit(0);
    	}
		int foundme = 0;
    	for ( i=0; i<MAX_FORWARDERS; i++ )
    	{
        	if ( ch->f_list[i] >= 0 && foundme == 1 )
        	{
				// assign mac dst for each packet
				 mac_->hdr_dst((char*) HDR_MAC(p), ch->f_list[i]);
				 break;
            }
            if ( ch->f_list[i] == mac_->addr() )    	foundme = 1;
        }
	}

    if ( ch->mac_type == 3 ) // AFR
 	{
     	// clear f_list first before manually assigning
     	for ( i=0; i<MAX_FORWARDERS; i++ )
     	{
     		ch->f_list[i] = -1;
     	}

     	int foundme = 0;
     	for ( i=0; i<MAX_FORWARDERS; i++ )
     	{
         	if ( ch->afr_f_list[i] >= 0 && foundme == 1 )
         	{
 				 mac_->hdr_dst((char*) HDR_MAC_RIPPLE_DATA(p), ch->afr_f_list[i]);
 	           	 ch->f_list[i] = ch->afr_f_list[i];
 				 break;
             }
             if ( ch->afr_f_list[i] == mac_->addr() )
             {
             	foundme = 1;
             	ch->f_list[i] = ch->afr_f_list[i];
             }
         }
 	}

    if ( ch->mac_type == 4 ) // EXOR
 	{
		// always use the last field as the MAC dst
     	for ( i=MAX_FORWARDERS-1; i>0; i-- )
     	{
         	 if ( ch->f_list[i] >= 0   )
         	 {
// 				 mac_->hdr_dst((char*) HDR_MAC_EXOR_DATA(p), ch->f_list[i]);
 				 break;
             }
        }
#ifdef TIANJI_NEW_DEBUG
     	printf("ll.cc, type=%d, mac dst=%d, addr=%d\n", ch->mac_type, ch->f_list[i], mac_->addr());
#endif
 	}



#else
	switch(ch->addr_type()) {

	case NS_AF_ILINK:
		mac_->hdr_dst((char*) HDR_MAC(p), ch->next_hop());
		break;

	case NS_AF_INET:
		dst = ch->next_hop();
		/* FALL THROUGH */

	case NS_AF_NONE:

		if (IP_BROADCAST == (u_int32_t) dst)
		{
			mac_->hdr_dst((char*) HDR_MAC(p), MAC_BROADCAST);
			break;
		}
		/* Assuming arptable is present, send query */
		if (arptable_) {
			tx = arptable_->arpresolve(dst, p, this);
			break;
		}
	default:

		int IPnh = (lanrouter_) ? lanrouter_->next_hop(p) : -1;
		if (IPnh < 0)
			mac_->hdr_dst((char*) HDR_MAC(p),macDA_);
		else if (varp_)
			tx = varp_->arpresolve(IPnh, p);
		else
			mac_->hdr_dst((char*) HDR_MAC(p), IPnh);
		break;
	}
#endif

	if (tx == 0) {
		Scheduler& s = Scheduler::instance();
	// let mac decide when to take a new packet from the queue.
		s.schedule(downtarget_, p, delay_);
	}
}



void LL::sendUp(Packet* p)
{
//printf("ll.cc sendUp, p uid=%d, mac addr=%d\n", hdr_cmn::access(p)->uid(), mac_->addr());
	Scheduler& s = Scheduler::instance();
	if (hdr_cmn::access(p)->error() > 0)
		drop(p);
	else
		s.schedule(uptarget_, p, delay_);
//		s.schedule(uptarget_, p, delay_+mac_->addr()*0.000001);
}


