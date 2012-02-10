/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
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
 *	This product includes software developed by the Computer Systems
 *	Engineering Group at Lawrence Berkeley Laboratory.
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
 * $Header: /tianji.cvsroot/newripple/ns-2.30/mac/mcexor/mac-mcexor.cc,v 1.6 2009/07/06 14:50:01 ripple Exp $
 *
 * Ported from CMU/Monarch's code, nov'98 -Padma.
 * Contributions by:
 *   - Mike Holland
 *   - Sushmita
 */

#include "delay.h"
#include "connector.h"
#include "packet.h"
#include "random.h"
#include "mobilenode.h"


#define TIANJI_NEW_DEBUG
//#define TIANJI_NEW_DEBUG_SHORT
//#define TIANJI_NEW_DEBUG_VERYSHORT
//#define TIANJI_DEBUG_ONLY_RECV_DOWN

#include "arp.h"
#include "ll.h"
#include "stdio.h"
#include "stdlib.h"
#include "mac-timers-mcexor.h"
#include "mac-mcexor.h"
#include "cmu-trace.h"

// Added by Sushmita to support event tracing
#include "agent.h"
#include "basetrace.h"




int mcexor_mac_data_hdr::offset_;
static class MacMCExORDataHeaderClass : public PacketHeaderClass {
public:
        MacMCExORDataHeaderClass() : PacketHeaderClass("PacketHeader/MacMCExORData",
                                             sizeof(mcexor_mac_data_hdr)) {
                bind_offset(&mcexor_mac_data_hdr::offset_);
        }
} class_mcexor_mac_data_hdr;

int mcexor_mac_ack_hdr::offset_;
static class MacMCExORAckHeaderClass : public PacketHeaderClass {
public:
        MacMCExORAckHeaderClass() : PacketHeaderClass("PacketHeader/MacMCExORAck",
                                             sizeof(mcexor_mac_ack_hdr)) {
                bind_offset(&mcexor_mac_ack_hdr::offset_);
        }
} class_mcexor_mac_ack_hdr;



inline void
MacMCExOR::checkBackoffTimer()
{
	if(is_idle() && mhBackoff_.paused())
		mhBackoff_.resume(phymib_.getDIFS());
	if(! is_idle() && mhBackoff_.busy() && ! mhBackoff_.paused())
		mhBackoff_.pause();
}

inline void
MacMCExOR::transmit(Packet *p, double timeout)
{
	struct hdr_cmn             *ch       = HDR_CMN(p);
	struct mcexor_mac_data_hdr *ddh      = HDR_MAC_MCEXOR_DATA(p);
	struct mcexor_mac_ack_hdr  *dah      = HDR_MAC_MCEXOR_ACK(p);
	PacketData                 *l_data;
	struct data_part           *l_data_p;
	Packet                     *l_NSpkt  = NULL;

	tx_active_  = 1;
	if (EOTtarget_) {
		assert (eotPacket_ == NULL);
		eotPacket_ = p->copy();
	}



	/*
	 * If I'm transmitting without doing CS, such as when
	 * sending an ACK, any incoming packet will be "missed"
	 * and hence, must be discarded.
	 */
	if(rx_state_ != MAC_IDLE) {
		//assert(dh->dh_fc.fc_type == MAC_Type_Control);
		//assert(dh->dh_fc.fc_subtype == MAC_Subtype_ACK);
		assert(pktRx_);
		ch->error() = 1;        /* force packet discard */
	}

	/*
	 * pass the packet on the "interface" which will in turn
	 * place the packet on the channel.
	 *
	 * NOTE: a handler is passed along so that the Network
	 *       Interface can distinguish between incoming and
	 *       outgoing packets.
	 */


#ifdef TIANJI_NEW_DEBUG
int dst, src;

if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) {
	src      = ETHER_ADDR(ddh->dh_ta);
	dst      = ETHER_ADDR(ddh->dh_ra);
	// get the pointer of pkts in this frame
	l_data = (PacketData *)p->userdata();
	l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
	l_NSpkt = l_data_p->NSpkt;
	printf("\n\n\nxmit in=%d, n=%d, puid=%d, retry=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ssrc_, ddh->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);
}
if ( ch->ptype() == PT_MAC_MCEXOR_ACK ) {
	src      = ETHER_ADDR(dah->dh_ta);
	dst      = ETHER_ADDR(dah->dh_ra);
	printf("\n\n\nxmit in=%d, n=%d, type=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), ssrc_, dah->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);
}


trace_pkt("xmit",p);
//trace_pkt("xmit",data_daemon);
#endif

#ifdef TIANJI_NEW_DEBUG_SHORT
int dst, src;

if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) {
	src      = ETHER_ADDR(ddh->dh_ta);
	dst      = ETHER_ADDR(ddh->dh_ra);
	// get the pointer of pkts in this frame
	l_data = (PacketData *)p->userdata();
	l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
	l_NSpkt = l_data_p->NSpkt;
	if ( index_==0 || index_==3)
		printf("\n\n\nxmit in=%d, n=%d, puid=%d, retry=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ssrc_, ddh->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);
	else
		printf("xmit in=%d, n=%d, puid=%d, retry=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ssrc_, ddh->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);

}
if ( ch->ptype() == PT_MAC_MCEXOR_ACK ) {
	src      = ETHER_ADDR(dah->dh_ta);
	dst      = ETHER_ADDR(dah->dh_ra);

	printf("xmit in=%d, n=%d, type=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), ssrc_, dah->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);
}

#endif

#ifdef TIANJI_NEW_DEBUG_VERYSHORT
int dst, src;

if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) {
	src      = ETHER_ADDR(ddh->dh_ta);
	dst      = ETHER_ADDR(ddh->dh_ra);
	// get the pointer of pkts in this frame
	l_data = (PacketData *)p->userdata();
	l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
	l_NSpkt = l_data_p->NSpkt;
	if ( index_==0 )
		printf("xmit in=%d, n=%d, puid=%d, retry=%d, size=%d, now=%lf\n",
			index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ssrc_, ch->size(),
			Scheduler::instance().clock());
}
#endif

    p_copy_for_tx = p->copy();
	downtarget_->recv(p_copy_for_tx, this);

	mhSend_.start(timeout);
	mhIF_.start(txtime(p));
}
inline void
MacMCExOR::setRxState(MacState newState)
{
	rx_state_ = newState;
	checkBackoffTimer();
}

inline void
MacMCExOR::setTxState(MacState newState)
{
	tx_state_ = newState;
	checkBackoffTimer();
}


/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class MacMCExORClass : public TclClass {
public:
	MacMCExORClass() : TclClass("Mac/MCExOR") {}
	TclObject* create(int, const char*const*) {
	return (new MacMCExOR());

}
} class_mac_mcexor;


/* ======================================================================
   Mac  and Phy MIB Class Functions
   ====================================================================== */

PHY_MIB_MCEXOR::PHY_MIB_MCEXOR(MacMCExOR *parent)
{
	/*
	 * Bind the phy mib objects.  Note that these will be bound
	 * to Mac/802_11 variables
	 */

	parent->bind("CWMin_", &CWMin);
	parent->bind("CWMax_", &CWMax);
	parent->bind("SlotTime_", &SlotTime);
	parent->bind("SIFS_", &SIFSTime);
	parent->bind("PreambleLength_", &PreambleLength);
	parent->bind("PLCPHeaderLength_", &PLCPHeaderLength);
	parent->bind_bw("PLCPDataRate_", &PLCPDataRate);

	parent->bind ("BER_",          &parent->BER);
	parent->bind ("num_sta_",      &parent->num_sta);
	parent->bind ("MAX_Sq_SIZE_",  &parent->MAX_Sq_SIZE);
	parent->bind_bw ("basicRate_", &basicRate);

}

MAC_MIB_MCEXOR::MAC_MIB_MCEXOR(MacMCExOR *parent)
{
	/*
	 * Bind the phy mib objects.  Note that these will be bound
	 * to Mac/802_11 variables
	 */

	parent->bind("RTSThreshold_", &RTSThreshold);
	parent->bind("ShortRetryLimit_", &ShortRetryLimit);
	parent->bind("LongRetryLimit_", &LongRetryLimit);
}

/* ======================================================================
   Mac Class Functions
   ====================================================================== */
MacMCExOR::MacMCExOR() :
	Mac(), phymib_(this), macmib_(this), mhIF_(this), mhNav_(this),
	mhRecv_(this), mhSend_(this),
	mhDefer_(this), mhBackoff_(this)
{

	nav_ = 0.0;
	tx_state_ = rx_state_ = MAC_IDLE;
	tx_active_ = 0;
	eotPacket_ = NULL;
	pktRTS_ = 0;
	pktCTRL_ = 0;
	cw_ = phymib_.getCWMin();
	ssrc_ = slrc_ = 0;
	// Added by Sushmita
        et_ = new EventTrace();

	sta_seqno_ = 1;
	cache_ = 0;
	cache_node_count_ = 0;

	// chk if basic/data rates are set
	// otherwise use bandwidth_ as default;

	Tcl& tcl = Tcl::instance();
	tcl.evalf("Mac/ExOR set basicRate_");
	if (strcmp(tcl.result(), "0") != 0)
		bind_bw("basicRate_", &basicRate_);
	else
		basicRate_ = bandwidth_;

	tcl.evalf("Mac/ExOR set dataRate_");
	if (strcmp(tcl.result(), "0") != 0)
		bind_bw("dataRate_", &dataRate_);
	else
		dataRate_ = bandwidth_;

	bind_bool("bugFix_timer_", &bugFix_timer_);

	EOTtarget_ = 0;
	bss_id_ = IBSS_ID;

	cur_Sq_size = 0;
	cur_Rq_size = 0;
	cur_correct_in_Rq = 0;

	Sq_head = NULL;
	Rq_head = NULL;

	if (MAX_FORWARDERS == 2) printf("MAX_FORWARDERS==2 means we are doing AFR now!!\n");
	if (MAX_FORWARDERS == 6) printf("MAX_FORWARDERS==6 means we are doing MCEXOR now!!\n");

	data_daemon = construct_data_daemon();
	ack_daemon  = construct_ack_daemon();
}


int
MacMCExOR::command(int argc, const char*const* argv)
{
	if (argc == 3) {
		if (strcmp(argv[1], "eot-target") == 0) {
			EOTtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if (EOTtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if (strcmp(argv[1], "bss_id") == 0) {
			bss_id_ = atoi(argv[2]);
			return TCL_OK;
		} else if (strcmp(argv[1], "log-target") == 0) {
			logtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if(logtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if(strcmp(argv[1], "nodes") == 0) {
			if(cache_) return TCL_ERROR;
			cache_node_count_ = atoi(argv[2]);
			cache_ = new Host[cache_node_count_ + 1];
			assert(cache_);
			bzero(cache_, sizeof(Host) * (cache_node_count_+1 ));
			return TCL_OK;
		} else if(strcmp(argv[1], "eventtrace") == 0) {
			// command added to support event tracing by Sushmita
                        et_ = (EventTrace *)TclObject::lookup(argv[2]);
                        return (TCL_OK);
                }
	}
	return Mac::command(argc, argv);
}

// Added by Sushmita to support event tracing
void MacMCExOR::trace_event(char *eventtype, Packet *p)
{
        if (et_ == NULL) return;
        char *wrk = et_->buffer();
        char *nwrk = et_->nbuffer();


        struct mcexor_mac_data_hdr* dh = HDR_MAC_MCEXOR_DATA(p);

	if(wrk != 0) {
		sprintf(wrk, "E -t "TIME_FORMAT" %s %2x ",
			et_->round(Scheduler::instance().clock()),
                        eventtype,
                        //ETHER_ADDR(dh->dh_sa)
                        ETHER_ADDR(dh->dh_ta)
                        );
        }
        if(nwrk != 0) {
                sprintf(nwrk, "E -t "TIME_FORMAT" %s %2x ",
                        et_->round(Scheduler::instance().clock()),
                        eventtype,
                        //ETHER_ADDR(dh->dh_sa)
                        ETHER_ADDR(dh->dh_ta)
                        );
        }
        et_->dump();
}

/* ======================================================================
   Debugging Routines
   ====================================================================== */
void
MacMCExOR::trace_pkt(char *fname, Packet *p)
{
	struct hdr_cmn                 * ch        = HDR_CMN(p);
	struct mcexor_mac_data_hdr     * ddh       = HDR_MAC_MCEXOR_DATA(p);
	struct mcexor_mac_ack_hdr      * dah       = HDR_MAC_MCEXOR_ACK(p);
	PacketData                     * l_data;
	struct data_part               * l_data_p;
	Packet                         * l_NSpkt   = NULL;
	struct hdr_cmn                 * l_ch;
	int                              src;
	int                              dst;
	int                              temp_dur;

	if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) {
		src      = ETHER_ADDR(ddh->dh_ta);
		dst      = ETHER_ADDR(ddh->dh_ra);
		temp_dur = ddh->dh_duration;
	}else if ( ch->ptype() == PT_MAC_MCEXOR_ACK ) {
		src      = ETHER_ADDR(dah->dh_ta);
		dst      = ETHER_ADDR(dah->dh_ra);
		temp_dur = dah->dh_duration;
    }else {
    	printf("MacMCExOR::trace_pkt, wrong ptype=%s, in=%d\n", packet_info.name(ch->ptype()), index_);
    	exit(0);
    }


	fprintf(stdout, "%s --- (INDEX: %d, time: %2.9f, addr=%p)\n", fname, index_, Scheduler::instance().clock(), p);
	fprintf(stdout, "\t dur/src/dst/type/size/n:%d %d %d %s %d %d\n",
			 temp_dur, src, dst, packet_info.name(ch->ptype()), ch->size(), ch->num_pkts());
	fprintf(stdout, "\t f.list: %d %d %d %d %d %d %d %d \n",
			ch->f_list[0], ch->f_list[1], ch->f_list[2],
			ch->f_list[3], ch->f_list[4], ch->f_list[5], ch->f_list[6], ch->f_list[7]);


		if ( ch->ptype() == PT_MAC_MCEXOR_ACK )
		 fprintf(stdout, "\t pID: %d \n", dah->mcexor_ack_hdr.pID );

	l_data = (PacketData *)p->userdata();
	for ( int i=0; i<ch->num_pkts(); i++) {
			l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
			l_NSpkt  = l_data_p->NSpkt;
			l_ch     = HDR_CMN(l_NSpkt);
			fprintf(stdout, "\t uid/es/ed/size:%d %d %d %d \n", l_ch->uid(), src, dst, l_ch->size() );
	}

}

void
MacMCExOR::dump(char *fname)
{
	fprintf(stdout,
		"%s --- (INDEX: %d, time: %2.9f)\n",
		fname, index_, Scheduler::instance().clock());

	fprintf(stdout,
		"\ttx_state_: %x, rx_state_: %x, \ttx_active_: %x, nav: %2.9f, idle: %d\n",
		tx_state_, rx_state_, tx_active_, nav_, is_idle());

	fprintf(stdout,
		"\tTx_: %lx, Rx_: %lx, d_daemon: %lx, a_daemon: %lx,  CTRL_: %lx, callback: %lx\n",
		(long) pktTx_, (long) pktRx_, (long) data_daemon,(long) ack_daemon,
		(long) pktCTRL_, (long) callback_);

	fprintf(stdout,
		"\tDefer: %d(exp %f), Backoff: %d (%d, exp %f), Recv: %d, Send: %d If: %d Nav: %d \n",
		mhDefer_.busy(), mhDefer_.expire(), mhBackoff_.busy(), mhBackoff_.paused(), mhBackoff_.expire(),
		mhRecv_.busy(), mhSend_.busy(), mhIF_.busy(), mhNav_.busy());
}


/* ======================================================================
   Packet Headers Routines
   ====================================================================== */
inline int
MacMCExOR::hdr_dst(char* hdr, int dst )
{
	struct mcexor_mac_data_hdr *dh = (struct mcexor_mac_data_hdr *) hdr;

       if (dst > -2) {
               if ((bss_id() == ((int)IBSS_ID)) || (addr() == bss_id())) {
                       /* if I'm AP (2nd condition above!), the dh_3a
                        * is already set by the MAC whilst fwding; if
                        * locally originated pkt, it might make sense
                        * to set the dh_3a to myself here! don't know
                        * how to distinguish between the two here - and
                        * the info is not critical to the dst station
                        * anyway!
                        */
                       STORE4BYTE(&dst, (dh->dh_ra));
               } else {
                       /* in BSS mode, the AP forwards everything;
                        * therefore, the real dest goes in the 3rd
                        * address, and the AP address goes in the
                        * destination address
                        */
                       STORE4BYTE(&bss_id_, (dh->dh_ra));
                       STORE4BYTE(&dst, (dh->dh_3a));
               }
       }


       return (u_int32_t)ETHER_ADDR(dh->dh_ra);
}

inline int
MacMCExOR::hdr_src(char* hdr, int src )
{
	struct mcexor_mac_data_hdr *dh = (struct mcexor_mac_data_hdr*) hdr;
        if(src > -2)
               STORE4BYTE(&src, (dh->dh_ta));
        return ETHER_ADDR(dh->dh_ta);
}

inline int
MacMCExOR::hdr_type(char* hdr, u_int16_t type)
{
	struct mcexor_mac_data_hdr *dh = (struct mcexor_mac_data_hdr*) hdr;
	if(type)
		STORE2BYTE(&type,(dh->dh_body));
	return GET2BYTE(dh->dh_body);
}


/* ======================================================================
   Misc Routines
   ====================================================================== */
inline int
MacMCExOR::is_idle()
{
	if(rx_state_ != MAC_IDLE)
		return 0;
	if(tx_state_ != MAC_IDLE)
		return 0;
	if(nav_ > Scheduler::instance().clock())
		return 0;

	return 1;
}

void
MacMCExOR::discard(Packet *p, const char* why)
{
	mcexor_mac_data_hdr* mh = HDR_MAC_MCEXOR_DATA(p);
	hdr_cmn *ch = HDR_CMN(p);

	/* if the rcvd pkt contains errors, a real MAC layer couldn't
	   necessarily read any data from it, so we just toss it now */
	if(ch->error() != 0) {
		return;
	}

	switch(mh->dh_fc.fc_type) {
	case MAC_Type_Management:
		return;
	case MAC_Type_Control:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_RTS:
			 if((u_int32_t)ETHER_ADDR(mh->dh_ta) ==  (u_int32_t)index_) {
				return;
			}
			/* fall through - if necessary */
		case MAC_Subtype_CTS:
		case MAC_Subtype_ACK:
			if((u_int32_t)ETHER_ADDR(mh->dh_ra) == (u_int32_t)index_) {
				return;
			}
			break;
		default:
			fprintf(stderr, "invalid MAC Control subtype\n");
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_Data:
			if((u_int32_t)ETHER_ADDR(mh->dh_ra) == \
                           (u_int32_t)index_ ||
                          (u_int32_t)ETHER_ADDR(mh->dh_ta) == \
                           (u_int32_t)index_ ||
                          ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
                                return;
			}
			break;
		default:
			fprintf(stderr, "invalid MAC Data subtype\n");
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "invalid MAC type (%x)\n", mh->dh_fc.fc_type);
		exit(1);
	}
}

void
MacMCExOR::capture(Packet *p)
{
	/*
	 * Update the NAV so that this does not screw
	 * up carrier sense.
	 */
	set_nav(usec(phymib_.getEIFS() + txtime(p)));
}

void
MacMCExOR::collision(Packet *p)
{
//printf("collision, can this happen?? in=%d\n", index_);
	switch(rx_state_) {
	case MAC_RECV:
		setRxState(MAC_COLL);
		/* fall through */
	case MAC_COLL:
		assert(pktRx_);
		assert(mhRecv_.busy());
		/*
		 *  Since a collision has occurred, figure out
		 *  which packet that caused the collision will
		 *  "last" the longest.  Make this packet,
		 *  pktRx_ and reset the Recv Timer if necessary.
		 */
		if(txtime(p) > mhRecv_.expire()) {
			mhRecv_.stop();
			Packet::free(pktRx_);
			pktRx_ = p;
			mhRecv_.start(txtime(pktRx_));
		}
		else {
			Packet::free(p);
			p = NULL;
		}
		break;
	default:
		assert(0);
	}
}

void
MacMCExOR::tx_resume(int priority)
{
	double rTime;
	assert(mhSend_.busy() == 0);
	assert(mhDefer_.busy() == 0);
#ifdef TIANJI_NEW_DEBUG
	printf("entering tx_resume, priority=%d, in=%d\n", priority, index_);
	dump("entering tx_resume");
#endif
	if(pktCTRL_) {
		if (!mhDefer_.busy()) {
        	mhDefer_.start((double)priority*phymib_.getSIFS());
		}
    } else if(pktRTS_) {
		if (mhBackoff_.busy() == 0) {
			if (bugFix_timer_) {
				mhBackoff_.start(cw_, is_idle(),
						 phymib_.getDIFS());
			}
			else {
				rTime = (Random::random() % cw_) *
					phymib_.getSlotTime();
				mhDefer_.start( phymib_.getDIFS() + rTime);
			}
		}
	} else if(pktTx_) {
		if (mhBackoff_.busy() == 0) {
			hdr_cmn *ch = HDR_CMN(pktTx_);
			struct mcexor_mac_data_hdr *mh = HDR_MAC_MCEXOR_DATA(pktTx_);

			if ((u_int32_t) ch->size() < macmib_.getRTSThreshold() || ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
				if (bugFix_timer_){ // RIPPE goes here
#ifdef TIANJI_NEW_DEBUG
					printf("cccccccccccc, is idle=%d, priority=%d, in=%d, bo time=%lf.\n",
							is_idle(), priority, index_, phymib_.getDIFS() );
					dump("cccccccccccccc, dumping");
#endif
					if ( is_idle() ) {
						mhBackoff_.start(cw_, is_idle(), phymib_.getDIFS() );
					} else {
						setTxState(MAC_IDLE);
						setRxState(MAC_IDLE);
						mhBackoff_.start(cw_, is_idle(),
								phymib_.getDIFS());
						if (!pktCTRL_ && mhDefer_.busy())
							mhDefer_.stop();
					}
				} else {
					rTime = (Random::random() % cw_) * phymib_.getSlotTime();
					mhDefer_.start(phymib_.getDIFS() +   rTime);
				}
            } else  {
#ifdef TIANJI_NEW_DEBUG
				printf("ddddddddddd  \n" );
#endif
            	mhDefer_.start(phymib_.getSIFS());
            }
		} else {
			mhBackoff_.stop();
			setTxState(MAC_IDLE);
#ifdef TIANJI_NEW_DEBUG
			printf("eeeeeeeeeeeeee, is idle=%d, \n", is_idle());
#endif
			mhBackoff_.start(cw_, is_idle(),
					phymib_.getDIFS() +
					(double)(priority)*(phymib_.getSIFS()));
		}

	} else if(callback_) {
		Handler *h = callback_;
		callback_ = 0;
		h->handle((Event*) 0);
	}
	setTxState(MAC_IDLE);
#ifdef TIANJI_NEW_DEBUG
	dump("leaving tx_resume......");
#endif
}

void
MacMCExOR::rx_resume()
{
	assert(pktRx_ == 0);
	assert(mhRecv_.busy() == 0);
	setRxState(MAC_IDLE);
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
MacMCExOR::backoffHandler()
{
#ifdef TIANJI_NEW_DEBUG
printf("MacMCExOR::backoffHandler, %d\n", index_);
dump("backoffHandler");
#endif
	// this should be used by the dst to send back MAC ACKs
	if(pktCTRL_) {
		assert(mhSend_.busy() || mhDefer_.busy());

		struct hdr_cmn           * ch       = HDR_CMN(pktCTRL_);
		int                       src       = pkt_e2esrc(pktCTRL_);

		if ( ch->ptype() == PT_MAC_MCEXOR_ACK && src == index_ ) {
			check_pktCTRL();
		}
		return;
	}

	if(check_pktRTS() == 0)
		return;

	if(check_pktTx() == 0)
		return;
}

void
MacMCExOR::deferHandler()
{
#ifdef TIANJI_NEW_DEBUG
dump("deferHandler");
printf("MacMCExOR::deferHandler, now=%lf, %d\n", Scheduler::instance().clock(),index_);
#endif
	assert(pktCTRL_ || pktRTS_ || pktTx_);

	if(check_pktCTRL() == 0)
		return;
	assert(mhBackoff_.busy() == 0);
	if(check_pktRTS() == 0)
		return;
//	if(check_pktTx() == 0)
//		return;
}

void
MacMCExOR::navHandler()
{
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::navHandler, %d\n", index_);
#endif
	if(is_idle() && mhBackoff_.paused())
		mhBackoff_.resume(phymib_.getDIFS());
}

void
MacMCExOR::recvHandler()
{
	recv_timer();
}

void
MacMCExOR::sendHandler()
{
	send_timer();
}


void
MacMCExOR::txHandler()
{
	if (EOTtarget_) {
		assert(eotPacket_);
		EOTtarget_->recv(eotPacket_, (Handler *) 0);
		eotPacket_ = NULL;
	}
	tx_active_ = 0;
#ifdef TIANJI_NEW_DEBUG
//printf("MacMCExOR::txHandler, in=%d\n", index_);
//if ( index_ == 0 && pktTx_ )
//	trace_pkt("trace pktTx_ txHandler",pktTx_);
#endif
}


/* ======================================================================
   The "real" Timer Handler Routines
   ====================================================================== */
void
MacMCExOR::send_timer()
{
#ifdef TIANJI_NEW_DEBUG
printf("send_timer, now=%lf, %d\n", Scheduler::instance().clock(), index_);
//if (index_==0){
//	print_mac_queue("send_timer, ", Sq_head);
//	trace_pkt("send_timer", data_daemon);
//}
#endif

	int temp_hops = 1;

    switch(tx_state_) {
		case MAC_CTS:
			assert(pktCTRL_);
			pktCTRL_ = 0;
			break;
		case MAC_SEND:
			if ( rx_state_ != MAC_IDLE  ) {
#ifdef TIANJI_NEW_DEBUG
				printf("RetransmitData, MAC ACK is lost, in=%d\n", index_);
#endif
			}

			// src retx
			if ( pktTx_ && ETHER_ADDR(HDR_MAC_MCEXOR_DATA(pktTx_)->dh_ta) == index_ )
				RetransmitDATA();
			else {
			    printf("send_timer, only src can be  here, in=%d\n", index_);
			    exit(0);
			}
			break;

		case MAC_ACK:
			assert(pktCTRL_);
#ifdef TIANJI_NEW_DEBUG
//    printf("send_timer, Sent an pktCTRL_, priority=%d, in=%d\n", hops_to_dst_ack(pktCTRL_)-1, index_);
    dump("send_timer, after sending a MAC ACK");
//    trace_pkt("send_timer pktCTRL_", pktCTRL_);
#endif

			// relay a data frame
			if ( pktTx_ ) {
				temp_hops = hops_to_dst(pktTx_);
			}


			if ( pktCTRL_ ) {
//				temp_hops = hops_to_dst_ack(pktCTRL_);
				if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_MCEXOR_DATA ) {
					// forwarders: renew data if not empty
					data_daemon = reconstruct_data_daemon_free_NSpkts(); // free NS pkts in it too
					pktCTRL_ = 0;
				}else if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_MCEXOR_ACK ) {
					// src/forwarders: renew ack if not empty
					ack_daemon = reconstruct_ack_daemon();
					pktCTRL_ = 0;
				}
			}


			break;
		case MAC_IDLE:
			break;
		default:
			assert(0);
	}
#ifdef TIANJI_NEW_DEBUG
    printf("send_timer, temp_hops=%d, in=%d\n", temp_hops, index_);
    dump("send_timer");
#endif
	tx_resume(temp_hops);
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
MacMCExOR::check_pktCTRL()
{
	struct hdr_cmn             * ch       = HDR_CMN(pktCTRL_);
	double                       timeout;
    int                          temp;
    struct mcexor_mac_data_hdr   * mdh      = HDR_MAC_MCEXOR_DATA(pktCTRL_);
	struct mcexor_mac_ack_hdr    * mah      = HDR_MAC_MCEXOR_ACK(pktCTRL_);
	struct frame_control         temp_fc;
    int                          temp_dur;

    if( pktCTRL_ == 0 )	                               return -1;
	if(tx_state_ == MAC_CTS || tx_state_ == MAC_ACK)   return -1;

	if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) {
		// there may be more pkts available during backoff, pktCTRL_ == data_daemon before calling
		update_data_daemon_to_send( RIGHT );

		temp_fc  = mdh->dh_fc;
		temp_dur = mdh->dh_duration;
        temp     = hops_to_dst(pktCTRL_);
	}

	if ( ch->ptype() == PT_MAC_MCEXOR_ACK ) {
		temp_fc  = mah->dh_fc;
		temp_dur = mah->dh_duration;
        temp     = hops_to_dst_ack(pktCTRL_);
	}

	switch(temp_fc.fc_subtype) {
	/*
	 *  If the medium is not IDLE, don't send the CTS.
	 */
	case MAC_Subtype_CTS:
		if(!is_idle()) {
			Packet::free(pktCTRL_); pktCTRL_ = 0;
			return 0;
		}
		setTxState(MAC_CTS);
   		    break;
    case MAC_Subtype_ACK:
        setTxState(MAC_ACK);
        break;
    case MAC_Subtype_Data:
    	// MCEXOR always uses MAC_Subtype_Data for both MAC data and ACK frames
		setTxState(MAC_ACK);
		timeout = temp * (DSSS_MaxPropagationDelay + phymib_.getSIFS()) + sec(temp_dur);
		break;
	default:
		printf( "check_pktCTRL:Invalid MAC Control subtype, type=%d, in=%d. \n", temp_fc.fc_subtype, index_);
		exit(1);
	}

//#ifdef TIANJI_NEW_DEBUG
//	printf("MacMCExOR::check_pktCTRL, sending pktCTRL_, dir=%d, in=%d\n", ch->direction(), index_);
//#endif
	transmit(pktCTRL_, timeout);
	return 0;
}

int
MacMCExOR::check_pktRTS()
{
	struct mcexor_mac_data_hdr *mh;
	double timeout;

	assert(mhBackoff_.busy() == 0);

	if(pktRTS_ == 0)
 		return -1;
	mh = HDR_MAC_MCEXOR_DATA(pktRTS_);

 	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_RTS:
		if(! is_idle()) {
			inc_cw();
			mhBackoff_.start(cw_, is_idle());
			return 0;
		}
		setTxState(MAC_RTS);
		break;
	default:
		fprintf(stderr, "check_pktRTS:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktRTS_, timeout);


	return 0;
}

int
MacMCExOR::check_pktTx()
{
	struct mcexor_mac_data_hdr * mh;
	double                       timeout;
    int                          temp;

	assert(mhBackoff_.busy() == 0);

// there may be more pkts available during backoff
	update_data_daemon_to_send( WRONG );
//	trace_pkt("trace data_daemon in check_pktTx, 8888888888888", data_daemon);
	if ( HDR_CMN(data_daemon)->num_pkts() > 0 )
		pktTx_ = data_daemon;
	else
		pktTx_ = 0;

	if(pktTx_ == 0)
		return -1;

	mh = HDR_MAC_MCEXOR_DATA(pktTx_);

	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_Data:
		if(! is_idle()) {
			inc_cw();
			mhBackoff_.start(cw_, is_idle());
			return 0;
		}

		setTxState(MAC_SEND);

		if(ETHER_ADDR(mh->dh_ra) != MAC_BROADCAST) {
                        temp = hops_to_dst(pktTx_);
                        timeout = temp*2*DSSS_MaxPropagationDelay
                                + temp*2*phymib_.getSIFS()
                                + sec(mh->dh_duration);
#ifdef TIANJI_NEW_DEBUG
printf("check_pktTx, timeout=%lf, dur=%lf, hops_to_dst=%d, in=%d\n", timeout, sec(mh->dh_duration), temp, index_);
#endif
        } else			timeout = txtime(pktTx_);

		break;
	default:
		fprintf(stderr, "check_pktTx:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktTx_, timeout);
	return 0;
}

void
MacMCExOR::sendDATA(Packet *p)
{
	assert(pktTx_ == 0);

        if ( data_daemon!=NULL )
        {
                pktTx_ = data_daemon;
        }else
        {
                printf("MacMCExOR::sendDATA, no frame to send, quit...%d\n",index_);
                exit(0);
        }

}


void
MacMCExOR::RetransmitDATA()
{
	struct hdr_cmn *ch;
	struct mcexor_mac_data_hdr *mh;
	u_int32_t *rcount, thresh;
	assert(mhBackoff_.busy() == 0);

	assert(pktTx_);
	assert(pktRTS_ == 0);



	ch = HDR_CMN(pktTx_);
	mh = HDR_MAC_MCEXOR_DATA(pktTx_);
    int src = ETHER_ADDR(mh->dh_ta);

	/*
	 *  Broadcast packets don't get ACKed and therefore
	 *  are never retransmitted.
	 */
	if(ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
		pktTx_ = 0;

		/*
		 * Backoff at end of TX.
		 */
		rst_cw();
		mhBackoff_.start(cw_, is_idle());

		return;
	}

	macmib_.ACKFailureCount++;

	if((u_int32_t) ch->size() <= macmib_.getRTSThreshold()) {
                rcount = &ssrc_;
               thresh = macmib_.getShortRetryLimit();
        } else {
                rcount = &slrc_;
               thresh = macmib_.getLongRetryLimit();
        }

	(*rcount)++;

#ifdef TIANJI_NEW_DEBUG
	printf("Src Retx data, rcount=%d, in=%d. \n", *rcount, index_);
#endif

	if(*rcount >= thresh)
	{
		macmib_.FailedCount++;
		*rcount = 0;

#ifdef TIANJI_NEW_DEBUG
		printf("Retx data, retry out?? in=%d \n", index_);
#endif
		clear_failed_in_Sq(ch->num_pkts());

		rst_cw();

		data_daemon = reconstruct_data_daemon();
		pktTx_ = 0;

		if ( cur_Sq_size > 0 ){
			update_data_daemon_to_send( WRONG );
			pktTx_ = data_daemon;
			mhBackoff_.start(cw_, is_idle());
		} else if( mhBackoff_.busy() ) {
			mhBackoff_.stop();
		}

	}
	else
	{
        if ( src == index_ ) {
            if ( cur_Sq_size > ch->num_pkts()  ) update_data_daemon_to_send( WRONG  );
        }
        else {
        	printf("Retx not by src, something wrong, quit! in=%d\n",index_);
        	exit(0);
        }

		mh->dh_fc.fc_retry = 1;

		inc_cw();
		mhBackoff_.start(cw_, is_idle());
	}

	return;
}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
MacMCExOR::send(Packet *p, Handler *h)
{
	double rTime;
	struct mcexor_mac_data_hdr* dh = HDR_MAC_MCEXOR_DATA(p);

	EnergyModel *em = netif_->node()->energy_model();
	if (em && em->sleep()) {
		em->set_node_sleep(0);
		em->set_node_state(EnergyModel::INROUTE);
	}

	callback_ = h;
	sendDATA(p);

	/*
	 * Assign the data packet a sequence number.
	 */
	dh->dh_scontrol = sta_seqno_++;

	/*
	 *  If the medium is IDLE, we must wait for a DIFS
	 *  Space before transmitting.
	 */
#ifdef TIANJI_NEW_DEBUG
//	dump("send dump");
#endif
	if(mhBackoff_.busy() == 0) {
		if(is_idle()) {
			if (mhDefer_.busy() == 0) {
				/*
				 * If we are already deferring, there is no
				 * need to reset the Defer timer.
				 */
				if (bugFix_timer_) { // this is true by default
					 mhBackoff_.start(cw_, is_idle(), phymib_.getDIFS());
				} else {
					rTime = (Random::random() % cw_)* (phymib_.getSlotTime());
					mhDefer_.start(phymib_.getDIFS() + rTime);
				}
			}
		} else {
			/*
			 * If the medium is NOT IDLE, then we start
			 * the backoff timer. WHAT THE HELL DOES THIS MEAN??? TIANJI
			 */
			mhBackoff_.start(cw_, is_idle());
		}
	}
}

void
MacMCExOR::recv(Packet *p, Handler *h)
{
	struct hdr_cmn             *hdr   = HDR_CMN(p);
	struct mcexor_mac_data_hdr *mdh   = HDR_MAC_MCEXOR_DATA(p);
	struct mcexor_mac_ack_hdr  *mah   = HDR_MAC_MCEXOR_ACK(p);
	int                         dst;
	int                         src;
	int                         why_error = 8;

	if(hdr->direction() == hdr_cmn::DOWN) {
		hdr->ptype() = PT_MAC_MCEXOR_DATA;
		dst = ETHER_ADDR(mdh->dh_ra);
		src = ETHER_ADDR(mdh->dh_ta);
#ifdef TIANJI_NEW_DEBUG
printf("Recv down, in=%d, now=%lf, Sq_size=%d, forwarders=%d %d %d %d %d %d %d %d, size=%d, uid=%d, in=%d\n",
		index_, Scheduler::instance().clock(),cur_Sq_size,  hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4], hdr->f_list[5],hdr->f_list[6],hdr->f_list[7], hdr->size(), hdr->uid(), index_);
//dump("Recv down dumping");
#endif
#ifdef TIANJI_DEBUG_ONLY_RECV_DOWN
if (index_==0) printf("Recv down, in=%d, now=%lf, Sq_size=%d, forwarders=%d %d %d %d %d %d %d %d, size=%d, uid=%d, in=%d\n",
		index_, Scheduler::instance().clock(),cur_Sq_size,  hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4], hdr->f_list[5],hdr->f_list[6],hdr->f_list[7], hdr->size(), hdr->uid(), index_);
//dump("Recv down dumping");
#endif

  	    push_Sq(p); // drain the ifq and save in Sq
		daemon_handler = h;
		callback_      = h;

		update_data_daemon_to_send(WRONG);
		send(data_daemon, daemon_handler);

		if ( cur_Sq_size < MAX_Sq_SIZE ) {
			// fetch another pkt if any from the upper layer
			Handler *h_temp = h;
			h = 0;
			h_temp->handle((Event*) 0);
		}
		return;
	}



	if ( hdr->ptype() == PT_MAC_MCEXOR_DATA ) {
		dst = ETHER_ADDR(mdh->dh_ra);
		src = ETHER_ADDR(mdh->dh_ta);
	}
	if ( hdr->ptype() == PT_MAC_MCEXOR_ACK ) {
		dst = ETHER_ADDR(mah->dh_ra);
		src = ETHER_ADDR(mah->dh_ta);
	}


	if( tx_active_ && hdr->error() == 0) {
		hdr->error() = 1;
		why_error = 0;
	}

	if( am_i_in_f_list(p) == WRONG ) {
		hdr->error() = 1;
		why_error = 1;
	}

//  hidden collisions bewteen MAC data and ACK frames, should cancel the current MAC ACK
	if ( mhDefer_.busy() && hdr->ptype() == PT_MAC_MCEXOR_DATA ) {
		mhDefer_.stop();
		ack_daemon = reconstruct_ack_daemon();
		pktCTRL_ = 0;
	}


	// src/forwarders should cancel their tx if hear high priority tx
	if (  am_i_in_f_list(p) && hear_a_higher_pri_tx(p) == RIGHT ) {
		if( mhDefer_.busy() )	mhDefer_.stop();
		if ( pktCTRL_ ) {
	  	    if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_MCEXOR_DATA ){
	   	    	data_daemon = reconstruct_data_daemon_free_NSpkts();
	   	    	pktCTRL_ = 0;
	  	    }else if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_MCEXOR_ACK ){
	   	    	ack_daemon = reconstruct_ack_daemon();
	   	    	pktCTRL_ = 0;
	  	    }
		}
		hdr->error() = 1; //should not receive this incoming pkt
		why_error = 2;
	}

	if ( src > num_sta || dst > num_sta ) { // htp from 802.11
		hdr->error() = 1;
		why_error = 3;
	}

	// forwarders get an ACK,
	// 1. stop waiting to forward, as the incoming ACK means that the data frame has been
	// received by the destination.
	if ( (am_i_forwarder(p)==RIGHT && dst != index_) && hdr->ptype() == PT_MAC_MCEXOR_ACK  ) {
  	    if ( mhDefer_.busy() )
  	    	mhDefer_.stop();
  	}



#ifdef TIANJI_NEW_DEBUG
if ( hdr->ptype() == PT_MAC_MCEXOR_DATA )
    printf("Recv up data, forwarders=%d %d %d %d %d %d %d %d, now=%lf, size=%d, type=%d, why=%d, err=%d, in=%d\n",
    		hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4],
    		hdr->f_list[5], hdr->f_list[6], hdr->f_list[7], Scheduler::instance().clock(), hdr->size(),
    		mdh->dh_fc.fc_type, why_error,  hdr->error(), index_);

if ( hdr->ptype() == PT_MAC_MCEXOR_ACK )
    printf("Recv up ack, forwarders=%d %d %d %d %d %d %d %d, now=%lf,size=%d, type=%d, why=%d, err=%d, in=%d\n",
    		hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4],
    		hdr->f_list[5], hdr->f_list[6], hdr->f_list[7], Scheduler::instance().clock(),  hdr->size(),
    		mah->dh_fc.fc_type, why_error, hdr->error(), index_);

if (index_==0) {
 	if (Sq_head!=NULL) print_mac_queue("recv up Ack", Sq_head);
    if (pktTx_!=NULL) trace_pkt("recv up Ack", pktTx_);
}
#endif


    if(rx_state_ == MAC_IDLE) {
		setRxState(MAC_RECV);
		pktRx_ = p;
		mhRecv_.start(txtime(p));
	} else {
		if(pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
			capture(p);
		} else {
			collision(p);
		}
	}




}

void
MacMCExOR::recv_timer()
{
	hdr_cmn             *ch     = HDR_CMN(pktRx_);
	mcexor_mac_data_hdr *mdh    = HDR_MAC_MCEXOR_DATA(pktRx_);
	mcexor_mac_ack_hdr  *mah    = HDR_MAC_MCEXOR_ACK(pktRx_);
	u_int8_t            type    ;
	u_int8_t            subtype ;
	int                 dst, src;

	if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) {
		type    = mdh->dh_fc.fc_type;
		subtype = mdh->dh_fc.fc_subtype;
		dst     = ETHER_ADDR(mdh->dh_ra);
		src     = ETHER_ADDR(mdh->dh_ta);
	}
    if ( ch->ptype() == PT_MAC_MCEXOR_ACK )  {
    	type    = mah->dh_fc.fc_type;
    	subtype = mah->dh_fc.fc_subtype;
    	dst     = ETHER_ADDR(mah->dh_ra);
    	src     = ETHER_ADDR(mah->dh_ta);
    }

#ifdef TIANJI_NEW_DEBUG
printf("MacMCExOR::recv_timer, type=%d, subtype=%d, now=%lf, id=%d, in=%d\n",
		type, subtype, Scheduler::instance().clock(), ch->uid(), index_);
dump("recv_timer");
//if (index_==0) {
//	print_mac_queue("bbbbbbbbbbbbbbb recv_timer before", Sq_head);
//}
#endif
	assert(pktRx_);
	assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);

	if(tx_active_) {
    	Packet::free(pktRx_);
    	goto done;
	}

	if(rx_state_ == MAC_COLL) {
		set_nav(usec(phymib_.getEIFS()));
		Packet::free(pktRx_);
		goto done;
	}

	if( ch->error() == 1 ) {
		set_nav(usec(phymib_.getEIFS()));
		Packet::free(pktRx_);
		goto done;
	}

// src/dst are also forwarders in this function
    if ( am_i_in_f_list(pktRx_) == WRONG ) {
    	if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) set_nav(mdh->dh_duration);
    	if ( ch->ptype() == PT_MAC_MCEXOR_ACK ) set_nav(mah->dh_duration);
    	Packet::free(pktRx_);
    	goto done;
    }


	switch(type) {
	case MAC_Type_Management:
		Packet::free(pktRx_);
		goto done;
	case MAC_Type_Control:
		switch(subtype) {
		case MAC_Subtype_RTS:
			printf("ExOR does not support RTS, quit!\n");
			exit(0);
		case MAC_Subtype_CTS:
            printf("ExOR does not support CTS, quit!\n");
            exit(0);
		case MAC_Subtype_ACK:
            printf("ExOR does not support 802.11 MAC ACK, quit!\n");
            exit(0);
		default:
			fprintf(stderr,"recvTimer1:Invalid MAC Control Subtype %x\n", subtype);
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(subtype) {
		case MAC_Subtype_Data:
			recvDATA(pktRx_);
			break;
		default:
			fprintf(stderr, "recv_timer2:Invalid MAC Data Subtype %x\n", subtype);
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "recv_timer3:Invalid MAC Type %x\n", subtype);
		exit(1);
	}

#ifdef TIANJI_NEW_DEBUG
if (index_==0) {
	print_mac_queue("bbbbbbbbbbbbbbb recv_timer after", Sq_head);
}
#endif


 done:
	pktRx_ = 0;
	rx_resume();
}


/*
 * txtime()	- pluck the precomputed tx time from the packet header
 */
double
MacMCExOR::txtime(Packet *p)
{
	struct hdr_cmn *ch = HDR_CMN(p);
	double t = ch->txtime();
	if (t < 0.0) {
		drop(p, "XXX");
 		exit(1);
	}
	return t;
}


/*
 * txtime()	- calculate tx time for packet of size "psz" bytes
 *		  at rate "drt" bps
 */
double
MacMCExOR::txtime(double psz, double drt)
{

        double t;
        int datalen = (int)(psz * 8) + 22;
        int Ndbps = ((int)drt/1000000) * 4 ;

        div_t divt = div(datalen, Ndbps) ;
        if ( divt.rem> 0 ) // symbol time = 0.000004 us
                t =  0.000004 * (divt.quot+1)  + PHYheader;
        else
                t =  0.000004 * divt.quot + PHYheader;

	return(t);
}



void
MacMCExOR::recvDATA(Packet *p)
{
/*************************************************************
 *
 * Called by forwarders and destination
 * Input: p can be MAC data or MAC ACK format
 *
 *************************************************************/

        struct           hdr_cmn   *ch          = HDR_CMN(p);
        int                         pkttype     = ch->ptype();
        struct mcexor_mac_data_hdr *ddh         = HDR_MAC_MCEXOR_DATA(p);
        struct mcexor_mac_ack_hdr  *dah         = HDR_MAC_MCEXOR_ACK(p);
        Packet                     *l_NSpkt;
        int                        dst, src, size, priority, carried_num_pkt, howmany_correct = 0;
        int                        receive_array[MAX_FRAGMENTS];
        double                     per;
        double                     var;
        PacketData                 *l_data;
        struct data_part           *l_data_p;
        int 					   forwarder_or_not = am_i_forwarder(p);

        if ( pkttype == PT_MAC_MCEXOR_DATA ) {
        	dst = ETHER_ADDR(ddh->dh_ra);
            src = ETHER_ADDR(ddh->dh_ta);
        }
        if ( pkttype == PT_MAC_MCEXOR_ACK ) {
        	dst = ETHER_ADDR(dah->dh_ra);
        	src = ETHER_ADDR(dah->dh_ta);
        }

        carried_num_pkt = ch->num_pkts();

        if ( pkttype == PT_MAC_MCEXOR_DATA && carried_num_pkt <=0  ){
            printf("recvDATA, carried_num_pkt should not < 0 for data frames, quit! in=%d \n", index_);
            exit(0);
        }

        // 1. Dst/forwarder gets a MCEXOR data, should send MAC ACKs sequentially
        if ( (dst == index_ || forwarder_or_not == RIGHT ) && pkttype == PT_MAC_MCEXOR_DATA ) {

        	// MAC hdr for this data frame is lost due to errors
            per = 1 - pow ( 1-BER, mac_data_size()*8 );
            var = Random::uniform(0.0,1.0);
            if (var <= per) {
            	set_nav(usec(phymib_.getEIFS()));
    			Packet::free(p);
#ifdef TIANJI_NEW_DEBUG
    			printf("recvDATA, mac hdr is lost due to channel noise, in=%d\n", index_);
#endif
        		goto done;
            }


        	for ( int i=0; i<MAX_FRAGMENTS; i++) {
        		receive_array[i] = WRONG;
        	}

#ifdef TIANJI_NEW_DEBUG
        	print_mac_queue("before push Rq", Rq_head);
#endif


        	// some incoming packets are not being retransmitted by the source, I should
            // drop them too without passing to the upper layer
        	l_data   = (PacketData *)p->userdata();
            l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
            l_NSpkt  = l_data_p->NSpkt;
        	struct mac_queue *l_queue = Rq_head;

			while ( l_queue != NULL ) {
#ifdef TIANJI_NEW_DEBUG
  printf("jjjjjjjjjjjjjjj, in puid=%d, Rq puid=%d, in=%d \n",  HDR_CMN(l_NSpkt)->uid(),l_queue->index, index_);
  print_mac_queue("jjjjjjjjjjjj", Rq_head);
#endif
			  if (  l_queue->index < HDR_CMN(l_NSpkt)->uid() ) {
				  if ( l_queue->flag == RIGHT ) cur_correct_in_Rq --;
				  cur_Rq_size --;
				  l_queue = l_queue->next;
				  Packet::free(Rq_head->NSpkt);
				  Rq_head->NSpkt = NULL;
				  free(Rq_head);
				  Rq_head = l_queue;
			  } else 	break;
			}



            howmany_correct = 0;
            for ( int i=0; i<carried_num_pkt; i++) {
                l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
                l_NSpkt  = l_data_p->NSpkt;
                size     = HDR_CMN(l_NSpkt)->size();

                per = 1 - pow ( 1-BER, size*8 );
                var = Random::uniform(0.0,1.0);
                if (var <= per) { // corrupted
					push_Rq(l_NSpkt, WRONG, TAIL, forwarder_or_not);
                }else{
					howmany_correct ++;
					push_Rq(l_NSpkt, RIGHT, TAIL, forwarder_or_not);
					receive_array[i] = RIGHT;
                }
            }

#ifdef TIANJI_NEW_DEBUG
    		printf("receive_array: ");
        	for ( int i=0; i<MAX_FRAGMENTS; i++) {
        		printf("%d ",receive_array[i]);
        	}
    		printf("\n");
        	print_mac_queue("after push Rq", Rq_head);

        	printf("cur_correct_in_Rq=%d, in=%d\n", cur_correct_in_Rq, index_);
#endif



            if ( cur_correct_in_Rq > 0 ) {
                 update_ack_daemon_to_send( WRONG, p, receive_array );
                 pktCTRL_ = ack_daemon;
                 priority = hops_to_dst(p) + 1;
            }

#ifdef TIANJI_NEW_DEBUG
            printf("dst/forwarder gets a data, priority=%d, in=%d\n", priority, index_);
            dump("dst/forwarder gets a data, dumping");
            trace_pkt("traceing pktRx_", p);
#endif
            if ( dst == index_  && pkttype == PT_MAC_MCEXOR_DATA )
				recv_correct_pkts( uptarget_ );
			if ( am_i_forwarder(p) == RIGHT && pkttype == PT_MAC_MCEXOR_DATA )
           		recv_all_pkts( uptarget_ );

        }

        // 2. Src/forwarder gets a MCEXOR ACK, updates Sq according to the ACK bitmap
        if ( (dst == index_ || forwarder_or_not ) && pkttype == PT_MAC_MCEXOR_ACK) {
        	// this ACK is lost due to errors
            size            = ch->size();
            per = 1 - pow ( 1-BER, size*8 );
            var = Random::uniform(0.0,1.0);
            if (var <= per) {
#ifdef TIANJI_NEW_DEBUG
            	printf("eeeeeeeeeeeeee, src %d gets a corrupte MAC ACK l=%d! var=%lf, per=%lf. \n",index_, size, var, per);
#endif
            	set_nav(usec(phymib_.getEIFS()));
    			Packet::free(p);
        		goto done;
            }


            // clear previous data frame
			if ( cur_Sq_size > 0 ) {
				update_flags_in_Sq(p);
				clear_ACKED_in_Sq(); // free NS pkts too
			}
			if ( pktTx_ != 0) {
#ifdef TIANJI_NEW_DEBUG
				printf("recvData, may be it is here?????????\n");
				trace_pkt("may be", pktTx_);
#endif
				// reset rcountb
				ssrc_ = 0;
				slrc_ = 0;
				data_daemon = reconstruct_data_daemon();
				pktTx_ = 0;
			}


			// check if there are new data
			if ( cur_Sq_size > 0 ){
				update_data_daemon_to_send( WRONG );
				if ( HDR_CMN(data_daemon)->num_pkts() ) pktTx_ = data_daemon;
			}


			if ( forwarder_or_not==RIGHT && pkttype == PT_MAC_MCEXOR_ACK ) {
				Packet::free(p);
				if ( mhBackoff_.busy() )   mhBackoff_.stop();
	            rst_cw();
	    		if ( mhSend_.busy() ) {
	    			mhSend_.stop();
					if (pktTx_)
						tx_resume(hops_to_dst(pktTx_));
					else
						tx_resume(-1);
	    		}
				goto done;
			}

	        if ( dst == index_ && pkttype == PT_MAC_MCEXOR_ACK)
				priority = hops_to_src_ack(p);

			if ( mhBackoff_.busy() )   mhBackoff_.stop();
            rst_cw();
        }

		if ( mhSend_.busy() ) mhSend_.stop();


		ch->num_forwards() += 1;

		if(dst != MAC_BROADCAST) {
			if(size >= (int)macmib_.getRTSThreshold()) {
				if (tx_state_ == MAC_CTS) {
					assert(pktCTRL_);
					pktCTRL_ = 0;
					mhSend_.stop();
					/*
					 * Our CTS got through.
					 */
				} else {
					Packet::free(p);
					return;
				}
				tx_resume(-1);
			} else {
				// ExOR goes here!!!
				tx_resume(priority);
			}
		}

done:

		// free used memory
		if ( dst == index_ && pkttype == PT_MAC_MCEXOR_DATA ) {
			// 1. Dst gets a MCEXOR data, free p, but do not free pkts in it
			Packet::freehdrs(p);
		}else if ( forwarder_or_not==RIGHT && pkttype == PT_MAC_MCEXOR_DATA ) {
			// 3. Forwarder gets a MCEXOR data, but does not free p
		} else {
			// 2. Src gets a MCEXOR ACK, free p
			// 4. Forwarder gets a MCEXOR ACK
			Packet::free(p);
		}
		p = NULL;
#ifdef TIANJI_NEW_DEBUG
		dump("recvData, leadving this funciton");
#endif
}






/**********************************************************************
 *
 *  ExOR scheme: Tianji Li, Jul 2008, Hamilton Institute, NUIM, Ireland
 *
 *
 **********************************************************************/


int
MacMCExOR::pkt_e2esrc(Packet *p)
{
        struct hdr_cmn *ch           = HDR_CMN(p);
        for ( int i=0; i<MAX_FORWARDERS; i++) {
            if ( ch->f_list[i] >= 0 ) {
                return ch->f_list[i];
            }
        }
        printf("MacRippple pkt_e2esrc, this should not happen!\n");
        exit(0);
}

int
MacMCExOR::pkt_e2edst(Packet *p)
{
        struct hdr_cmn *ch           = HDR_CMN(p);
        for ( int i=MAX_FORWARDERS-1; i>0; i--) {
            if ( ch->f_list[i] >= 0 ) {
                return ch->f_list[i];
            }
        }
        return -1;
}


void
MacMCExOR::push_Sq(Packet *p)
{
        hdr_cmn               *ch           = HDR_CMN(p);
        struct mac_queue      *l_queue;

        l_queue           = (struct mac_queue *) malloc(sizeof(struct mac_queue));
        l_queue->index    = ch->uid();
        l_queue->origsize = ch->size();
        l_queue->NSpkt    = p;
        l_queue->flag     = RIGHT;
        l_queue->next     = NULL;
        l_queue->e2esrc   = pkt_e2esrc(p);
        l_queue->e2edst   = pkt_e2edst(p);

        if ( cur_Sq_size==0 ) {
            Sq_head = l_queue;
            Sq_tail = l_queue;
        } else {
            Sq_tail->next = l_queue;
            Sq_tail = l_queue;
        }
        cur_Sq_size ++;

        return;
}



void
MacMCExOR::push_Rq(Packet *p, int flag, int head_or_tail, int forwarding)
{
        hdr_cmn                    * ch          = HDR_CMN(p);
        int                          founded     = WRONG;
        struct mac_queue           * l_queue;
        struct mac_queue           * temp_queue, * temp_queue_before;

        if ( head_or_tail == TAIL ) { // i am the dst
            // this pkt may have already queued
            if ( Rq_head != NULL ) {
                temp_queue = Rq_head;
                while ( temp_queue != NULL ) {
                    if ( temp_queue->index == ch->uid() ) {
                        if ( temp_queue->flag == WRONG ) {
                        	temp_queue->flag = RIGHT;
                        	cur_correct_in_Rq ++;
                        }
                        founded = RIGHT;
                        break;
                    }
                    temp_queue = temp_queue->next;
                }
            }

            if ( founded == RIGHT )
                return;

           // this is a new pkt
            l_queue           = (struct mac_queue *) malloc(sizeof(struct mac_queue));
            l_queue->index    = ch->uid();
            l_queue->origsize = ch->size();
            l_queue->NSpkt    = p;
            l_queue->flag     = flag;
            l_queue->next     = NULL;

            l_queue->e2esrc = pkt_e2esrc(p);
            l_queue->e2edst = pkt_e2edst(p);

            if ( cur_Rq_size==0 ) {
                Rq_head = l_queue;
                Rq_tail = l_queue;
            } else if ( cur_Rq_size == 1) {
            	if ( ch->uid() < Rq_head->index ) {
            		l_queue->next = Rq_head;
            		Rq_head = l_queue;
            	} else{
                    Rq_tail->next = l_queue;
                    Rq_tail = l_queue;
                }
            } else {
            	if ( ch->uid() < Rq_head->index ) {
            		l_queue->next = Rq_head;
            		Rq_head = l_queue;
            	} else {
                	temp_queue_before 	= Rq_head;
                	temp_queue 			= Rq_head;
                	founded 			= WRONG;
                    while ( temp_queue != NULL ) {
                        if ( ch->uid() < temp_queue->index  ) {
                            founded = RIGHT;
                            break;
                        }
                        temp_queue_before 	= temp_queue;
                        temp_queue 			= temp_queue->next;
                    }
                    if ( founded == RIGHT ) {
                    	l_queue->next 			= temp_queue;
                    	temp_queue_before->next = l_queue;
                    } else {
                    	Rq_tail->next 	= l_queue;
                    	Rq_tail 		= l_queue;
                    }
                }
            }
        }


        if ( forwarding == RIGHT ) cur_correct_in_Rq ++;
        if ( forwarding ==  WRONG  && flag == RIGHT ) cur_correct_in_Rq ++;
        cur_Rq_size ++;
        return;
}




void
MacMCExOR::update_data_daemon_to_send(  int forwarding )
{
        hdr_cmn                     * ch_daemon       = HDR_CMN(data_daemon);
        hdr_cmn                     * ch_pkt_in_Sq    = NULL;
        struct mcexor_mac_data_hdr  * mh_daemon       = HDR_MAC_MCEXOR_DATA(data_daemon);
        struct mcexor_mac_data_hdr  * mh_pkt_in_Sq    = NULL;
        struct mac_queue            * l_queue         = NULL;
        int                           daemon_size     = 0;
        int                           daemon_num_pkts = 0;
        int                           daemon_duration = 0;
        int                           daemon_dst;
        int                           temp, howmany_new;
        int                           addmachdrs      = WRONG;

        if ( ch_daemon->ptype() != PT_MAC_MCEXOR_DATA ) {
        	printf("update_data_daemon_to_send, MAC ACK should not be here, quit! in=%d. \n", index_);
        	exit(0);
        }

        if ( forwarding == WRONG ) {
        	// I am src, Sq != NULL
        	if ( Sq_head == NULL ) {
        		printf("update_data_daemon_to_send, src %d has no pkt to send, quit!\n", index_);
        		dump("1111111111");
        		trace_pkt("1111111111", data_daemon);
        		print_mac_queue("111111111", Sq_head);
        		exit(0);
        	}
            daemon_num_pkts = ch_daemon->num_pkts();
            daemon_size     = ch_daemon->size();
            daemon_dst      = pkt_e2edst(data_daemon);

#ifdef TIANJI_NEW_DEBUG
      if (daemon_num_pkts > MAX_FRAGMENTS  ) {
    	  printf("update_data_daemon_to_send, daemon_num_pkts > MAX_FRAGMENTS, quit!\n");
    	  print_mac_queue("000000000000", Sq_head);
    	  trace_pkt("000000000", data_daemon);
    	  exit(0);
      }
#endif

      		if ( daemon_num_pkts==0 ) addmachdrs = RIGHT;

            if ( daemon_num_pkts >= ltjmin(MAX_FRAGMENTS,cur_Sq_size) )
            	return;

            howmany_new  	= 0;
            l_queue   		= Sq_head;
            while ( l_queue != NULL ) {
            	if ( l_queue->flag != ACKED && ( daemon_dst < 0 || l_queue->e2edst == daemon_dst) )
            		howmany_new ++;
					l_queue = l_queue->next;
            }

            if ( howmany_new <= daemon_num_pkts ) {
            	return; // there are no new packets to aggregate
            }

            // move to the start of new packets that are i) not in data_daemon now, and ii) for a same dst
            l_queue   		= Sq_head;
            temp            = 0;
            while ( l_queue != NULL && temp < daemon_num_pkts ) {
            	if ( l_queue->flag != ACKED && ( daemon_dst < 0 || l_queue->e2edst == daemon_dst) ) temp ++;
			    l_queue = l_queue->next;
            }

            // l_queue is the start of the new packets
            while ( l_queue != NULL && daemon_num_pkts < MAX_FRAGMENTS) {
                if (  l_queue->flag != ACKED ) {
//                    if ( daemon_num_pkts == temp ) {
                       if ( daemon_num_pkts == 0 ) {
                           daemon_dst    = l_queue->e2edst;
                           mh_pkt_in_Sq  = HDR_MAC_MCEXOR_DATA(l_queue->NSpkt);
                           ch_pkt_in_Sq  = HDR_CMN(l_queue->NSpkt);
                       }
                       // only send pkts to a same direction
                       if ( daemon_dst == l_queue->e2edst ) {
                           daemon_size += l_queue->origsize + FRAG_FCS_LEN;
                           save_into_data_part(l_queue, daemon_num_pkts);
                           daemon_num_pkts ++;
                           l_queue->flag = UNACKED;
                       }
                }

                l_queue       = l_queue->next;
            }


            if ( mh_pkt_in_Sq != NULL )
              for ( int i=0; i<MAX_FORWARDERS; i++) {
                  ch_daemon->f_list[i] = ch_pkt_in_Sq->f_list[i];
              }
        }

        if ( forwarding == RIGHT ) {
            // I am a forwarder, search Sq for more pkts for the same dst, if the frame is not big enough
            // if no new packets or frame too long, quit
            daemon_num_pkts 		= ch_daemon->num_pkts();
            daemon_size     		= ch_daemon->size();
            daemon_dst      		= pkt_e2edst(data_daemon);

            if ( daemon_num_pkts==0 ) addmachdrs = RIGHT;

            if ( daemon_num_pkts >= ltjmin(MAX_FRAGMENTS,cur_Sq_size) ) {
            	goto update_hdrs;
            }

			l_queue = Sq_head;
			temp = daemon_num_pkts;
			for ( int i=0; i<temp; i++){
				l_queue = l_queue->next;
			}

            for ( int i=temp; i<ltjmin(MAX_FRAGMENTS,cur_Sq_size); i++) {
				if ( l_queue->flag != ACKED ){
				   // only forward pkts to a same direction
				   if ( daemon_dst == l_queue->e2edst ) {
					   daemon_size += l_queue->origsize + FRAG_FCS_LEN;
					   save_into_data_part(l_queue, daemon_num_pkts);
					   daemon_num_pkts ++;
					   l_queue->flag = UNACKED;
				   }
				}
				l_queue = l_queue->next;
			}
        }

update_hdrs:

        if (  addmachdrs == RIGHT  ) ch_daemon->size()  = daemon_size + mac_data_size();
        else                       ch_daemon->size()  = daemon_size;
        ch_daemon->txtime()    = txtime(ch_daemon->size(), dataRate_);
        ch_daemon->error()     = 0;
        ch_daemon->direction() = hdr_cmn::DOWN;

        mh_daemon->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
        mh_daemon->dh_fc.fc_type             = MAC_Type_Data;
        mh_daemon->dh_fc.fc_subtype          = MAC_Subtype_Data;

        temp                   = hops_to_dst(data_daemon);
        daemon_duration        = usec((double)temp*ch_daemon->txtime());
        mh_daemon->dh_duration = daemon_duration;
        ch_daemon->num_pkts()  = daemon_num_pkts;

        STORE4BYTE(&daemon_dst, (mh_daemon->dh_ra));
        STORE4BYTE(&index_,     (mh_daemon->dh_ta));

        return ;
}


void
MacMCExOR::update_ack_daemon_to_send( int forwarding, Packet *pkt_rcved, int rcv_array[MAX_FRAGMENTS] )
{
        hdr_cmn                     *  ch_ack_daemon    = HDR_CMN(ack_daemon);
        struct mcexor_mac_ack_hdr   *  mh_ack_daemon    = HDR_MAC_MCEXOR_ACK(ack_daemon);
        int                            tx_duration      = 0;
        int                            temp;

    	// I am the dst, config ack_daemon
        if ( forwarding == WRONG  ) {
            hdr_cmn   *  ch_pktRx_  = HDR_CMN(pkt_rcved);
            if ( Rq_head == NULL ) {
            	printf("update_ack_daemon_to_send, Rq_head should not be empty! in=%d \n ", index_);
            	exit(0);
            }
            // set dst for this MAC ACK
            for (int i=0; i<MAX_FORWARDERS; i++) {
                 if ( ch_pktRx_->f_list[i] >= 0 ) {
                	 ch_ack_daemon->next_hop_  = ch_pktRx_->f_list[i];
                     break;
                 }
            }

            // construct forwarding list
            for (int i=0;i<MAX_FORWARDERS;i++) {
            	ch_ack_daemon->f_list[i] = -1;
            }
            int j=0;
            for (int i=0;i<MAX_FORWARDERS;i++) {
                if ( ch_pktRx_->f_list[MAX_FORWARDERS-1-i] >= 0 ){
                	ch_ack_daemon->f_list[j] = ch_pktRx_->f_list[MAX_FORWARDERS-1-i];
                    j ++;
                }

            }


            // Construct ACK pID
            construct_ACK_pID( pkt_rcved, rcv_array  );
            for ( int i=0; i<MAX_FRAGMENTS; i++) {
            	mh_ack_daemon->mcexor_ack_hdr.pID = rq_ack_pID;
            }


            ch_ack_daemon->direction() = hdr_cmn::DOWN;
            ch_ack_daemon->txtime()    = txtime(ch_ack_daemon->size(), basicRate_);
            ch_ack_daemon->error()     = 0;

            temp                       = hops_to_dst_ack(ack_daemon);
            tx_duration                = usec((double)temp*ch_ack_daemon->txtime());
            mh_ack_daemon->dh_duration = tx_duration;

            STORE4BYTE(&ch_ack_daemon->next_hop_, (mh_ack_daemon->dh_ra));
            STORE4BYTE(&index_,                   (mh_ack_daemon->dh_ta));

        } else {
        	// I am a forwarder, update ack_daemon
            ch_ack_daemon->direction() = hdr_cmn::DOWN;
//            ch_ack_daemon->txtime()    = txtime(ch_ack_daemon->size(), dataRate_);
            ch_ack_daemon->txtime()    = txtime(ch_ack_daemon->size(), basicRate_);
            ch_ack_daemon->error()     = 0;

            temp                       = hops_to_dst_ack(ack_daemon);
            tx_duration                = usec((double)temp*ch_ack_daemon->txtime());
            mh_ack_daemon->dh_duration = tx_duration;

            STORE4BYTE(&ch_ack_daemon->next_hop_, (mh_ack_daemon->dh_ra));
            STORE4BYTE(&index_,                   (mh_ack_daemon->dh_ta));
        }
}


void
MacMCExOR::alloc_data_part_memory( Packet *p )
{
    if ( p->accessdata() != 0 )
    {
    	p->setdata(NULL);
    }
    p->allocdata(sizeof(struct data_part)*MAX_FRAGMENTS);
}

void
MacMCExOR::remove_data_part_content(Packet *p)
{
    if ( p->accessdata() != 0 )
    {
    	p->setdata(NULL);
    }
}

void
MacMCExOR::save_into_data_part(struct mac_queue *l_queue, int whichone)
{
    PacketData        *l_data;
    struct data_part  *l_data_p;

    l_data   = (PacketData *)data_daemon->userdata();
    l_data_p = (struct data_part *)l_data->access_data(whichone*sizeof(struct data_part));

    l_data_p->NSpkt = l_queue->NSpkt;
}




void
MacMCExOR::construct_ACK_pID( Packet *p, int rcv_array[MAX_FRAGMENTS] )
{
    struct mac_queue    * l_queue;

    rq_ack_pID = -1;

    l_queue = Rq_head;
    while ( l_queue != NULL ) {
		if ( l_queue->flag == RIGHT ) {
			rq_ack_pID = l_queue->index;
		} else
			return;
		l_queue = l_queue->next;
    }

    if ( rq_ack_pID < 0 ) printf("MacMCExOR::construct_ACK_pID, can this happen? in=%d. n", index_);
    return;
}



int
MacMCExOR::hear_a_higher_pri_tx(Packet *p)
{
        struct hdr_cmn              * ch              = HDR_CMN(p);
        struct mcexor_mac_data_hdr  * mdh             = HDR_MAC_MCEXOR_DATA(p);
        struct mcexor_mac_ack_hdr   * mah             = HDR_MAC_MCEXOR_ACK(p);
        int                           my_dist_to_dst  = hops_to_dst(p);
        int                           p_dist_to_dst   = -1;
        int                           src;

        if ( ch->ptype() == PT_MAC_MCEXOR_DATA ) src = ETHER_ADDR(mdh->dh_ta);
        if ( ch->ptype() == PT_MAC_MCEXOR_ACK )  src = ETHER_ADDR(mah->dh_ta);


        for ( int i=MAX_FORWARDERS-1; i>=0; i--) {
            if ( ch->f_list[i] >= 0 ) {
                p_dist_to_dst ++;
            }
            if ( ch->f_list[i] == src ) {
                break;
            }
        }
#ifdef TIANJI_NEW_DEBUG
       if (index_==1)
         printf("ggggggggggggg, my_dist_to_dst=%d, p_dist_to_dst=%d\n", my_dist_to_dst, p_dist_to_dst);
#endif
        if ( my_dist_to_dst > p_dist_to_dst )
                return RIGHT;

        return WRONG;
}




int
MacMCExOR::am_i_forwarder(Packet *p)
{
 //I can only be one of the forwarders
        struct hdr_cmn *ch         = HDR_CMN(p);

        for ( int i=1; i<MAX_FORWARDERS-1; i++) {
            if ( ch->f_list[i] == index_ && ch->f_list[i+1]!=-1 && ch->f_list[i-1]!=-1)
                return RIGHT;
        }
        return WRONG;
}



int
MacMCExOR::am_i_in_f_list(Packet *p)
{
//I can be the src/dst and one of the forwarders
        struct hdr_cmn *ch         = HDR_CMN(p);

        for ( int i=0; i<MAX_FORWARDERS; i++) {
            if ( ch->f_list[i] == index_ )
                return RIGHT;
        }
        return WRONG;
}

int
MacMCExOR::my_priority(Packet *p)
{
        struct hdr_cmn *ch        = HDR_CMN(p);
        int temp                  = 0;

        if ( am_i_forwarder(p) == RIGHT ) {
            for ( int i=MAX_FORWARDERS-1; i>=0; i--) {
                if ( ch->f_list[i] >= 0 ) {
                    temp ++;
                }
                if ( ch->f_list[i] == index_ ) {
                    return temp;
                }
            }
        }
        printf("MacMCExOR::my_priority, should not reach here! in=%d\n", index_);
        exit(0);
}

Packet *
MacMCExOR::reconstruct_data_daemon(  )
{
	 Packet::freehdrs(data_daemon);
	 return ( construct_data_daemon() );
}

Packet *
MacMCExOR::reconstruct_data_daemon_free_NSpkts(  )
{
	hdr_cmn * ch    = HDR_CMN(data_daemon);
	int     howmany = ch->num_pkts();

	clear_failed_in_Sq(howmany);
	Packet::freehdrs(data_daemon);
	return ( construct_data_daemon() );
}

Packet *
MacMCExOR::reconstruct_ack_daemon(  )
{
	 Packet::free(ack_daemon);
	 return ( construct_ack_daemon() );
}


Packet *
MacMCExOR::construct_data_daemon( )
{
            Packet                     * p     = Packet::alloc();
            hdr_cmn                    * ch    = HDR_CMN(p);
            struct mcexor_mac_data_hdr * mdh   = HDR_MAC_MCEXOR_DATA(p);

            alloc_data_part_memory (p);

            ch->ptype() = PT_MAC_MCEXOR_DATA;
            ch->iface() = -2;
            ch->error() = 0;

            for ( int i=MAX_FORWARDERS-1; i>0; i--) {
                ch->f_list[i] = -1 ;
            }

            mdh->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
            mdh->dh_fc.fc_type             = MAC_Type_Data;
            mdh->dh_fc.fc_subtype          = MAC_Subtype_Data;
            mdh->dh_fc.fc_to_ds            = 0;
            mdh->dh_fc.fc_from_ds          = 0;
            mdh->dh_fc.fc_more_frag        = 0;
            mdh->dh_fc.fc_retry            = 0;
            mdh->dh_fc.fc_pwr_mgt          = 0;
            mdh->dh_fc.fc_more_data        = 0;
            mdh->dh_fc.fc_wep              = 0;
            mdh->dh_fc.fc_order            = 0;

            return p;
}

Packet *
MacMCExOR::construct_ack_daemon( )
{
            Packet                    * p      = Packet::alloc();
            hdr_cmn                   * ch     = HDR_CMN(p);
            struct mcexor_mac_ack_hdr * mah    = HDR_MAC_MCEXOR_ACK(p);

            ch->ptype() = PT_MAC_MCEXOR_ACK;
            ch->iface() = -2;
            ch->error() = 0;
            ch->uid()   = -1;
            ch->size()  = mac_ack_size();

            mah->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
            mah->dh_fc.fc_type             = MAC_Type_Data;
            mah->dh_fc.fc_subtype          = MAC_Subtype_Data;
            mah->dh_fc.fc_to_ds            = 0;
            mah->dh_fc.fc_from_ds          = 0;
            mah->dh_fc.fc_more_frag        = 0;
            mah->dh_fc.fc_retry            = 0;
            mah->dh_fc.fc_pwr_mgt          = 0;
            mah->dh_fc.fc_more_data        = 0;
            mah->dh_fc.fc_wep              = 0;
            mah->dh_fc.fc_order            = 0;

            return p;
}

int
MacMCExOR::ltjmin(int a, int b)
{
    if ( a < b )
        return a;
    return b;
}


void
MacMCExOR::recv_correct_pkts(NsObject *uptarget)
{
    struct mac_queue *     l_queue;

    l_queue = Rq_head;
    while ( l_queue != NULL ){
        if ( l_queue->flag == RIGHT ) {
#ifdef TIANJI_NEW_DEBUG
hdr_cmn *hdr = HDR_CMN(l_queue->NSpkt);
printf("Finish recv in=%d, uid=%d, cur_correct_in_Rq=%d, forwarders=%d %d %d %d %d %d %d %d, mac_type=%d. \n",
		index_, hdr->uid(), cur_correct_in_Rq, hdr->f_list[0], hdr->f_list[1], hdr->f_list[2],
		hdr->f_list[3], hdr->f_list[4], hdr->f_list[5], hdr->f_list[6], hdr->f_list[7], HDR_CMN(l_queue->NSpkt)->mac_type);
#endif
            HDR_CMN(l_queue->NSpkt)->direction() = hdr_cmn::UP;
            uptarget->recv(l_queue->NSpkt, (Handler*) 0);
            remove_Rq_head( );
            cur_Rq_size --;
            cur_correct_in_Rq --;
        } else
            return; // only pass consecutively correct pkts to upper layer
        l_queue = l_queue->next;
    }
}


void
MacMCExOR::recv_all_pkts(NsObject *uptarget)
{
    struct mac_queue *     l_queue;

    l_queue = Rq_head;
    while ( l_queue != NULL ){
#ifdef TIANJI_NEW_DEBUG
hdr_cmn *hdr = HDR_CMN(l_queue->NSpkt);
printf("Finish recv in=%d, uid=%d, cur_correct_in_Rq=%d, forwarders=%d %d %d %d %d %d %d %d, mac_type=%d. \n",
		index_, hdr->uid(), cur_correct_in_Rq, hdr->f_list[0], hdr->f_list[1], hdr->f_list[2],
		hdr->f_list[3], hdr->f_list[4], hdr->f_list[5], hdr->f_list[6], hdr->f_list[7], HDR_CMN(l_queue->NSpkt)->mac_type);
#endif
		HDR_CMN(l_queue->NSpkt)->direction() = hdr_cmn::UP;
		uptarget->recv(l_queue->NSpkt, (Handler*) 0);
		remove_Rq_head( );
		cur_Rq_size --;
		cur_correct_in_Rq --;
        l_queue = l_queue->next;
    }
}

void
MacMCExOR::remove_Rq_head( )
{
        struct mac_queue * l_queue;

        if ( cur_Rq_size < 1) {
        	printf("MacMCExOR::remove_Rq_head, cur_Rq_size < 1, quit! in=%d\n", index_);
        	exit(0);
        }else if ( cur_Rq_size == 1 )
        {
                free(Rq_head);
                Rq_head = NULL;
                Rq_tail = NULL;
        }else
        {
                l_queue = Rq_head->next;
                free(Rq_head);
                Rq_head = l_queue;
        }
        return;
}


// remove all successfully transmitted pkts in the Rq
void
MacMCExOR::clear_ACKED_in_Rq( )
{
        struct mac_queue       * l_queue = NULL;

        if ( cur_Rq_size == 0 )
        {
          return;
        }else if ( cur_Rq_size == 1 )
        {
                    l_queue = Rq_head;
                    if ( l_queue->flag == ACKED )
                    {
printf("clear_ACKED_in_Rq, 111, does this happen? in=%d \n", index_);
                            free(Rq_head);
                            Rq_head = NULL;
                            Rq_tail = NULL;
                            cur_Rq_size = 0;
                    }
        }else {
            while ( Rq_head != NULL && Rq_head->flag == ACKED )
            {
printf("clear_ACKED_in_Rq, 222, does this happen? in=%d \n", index_);
                remove_Rq_head();
                cur_Rq_size --;
            }
        }
        return;
}



void
MacMCExOR::remove_Sq_head( )
{
        struct mac_queue * l_queue;

        if ( cur_Sq_size < 1) {
        	printf("MacMCExOR::remove_Sq_head, cur_Sq_size < 1, quit! in=%d\n", index_);
        	exit(0);
        }else  if ( cur_Sq_size == 1) {
        	Packet::free(Sq_head->NSpkt);
        	Sq_head->NSpkt = NULL;
            free(Sq_head);
            Sq_head = NULL;
            Sq_tail = NULL;
        }else{
            l_queue = Sq_head->next;
        	Packet::free(Sq_head->NSpkt);
        	Sq_head->NSpkt = NULL;
            free(Sq_head);
            Sq_head = l_queue;
        }
        cur_Sq_size --;
        return;
}


void
MacMCExOR::update_flags_in_Sq(Packet *p)
{
    if ( Sq_head == NULL ) return;

    struct mac_queue          * l_queue = Sq_head;
    struct mcexor_mac_ack_hdr * mh      = HDR_MAC_MCEXOR_ACK(p);


    for ( int i=0; i<MAX_FRAGMENTS; i++){
        l_queue = Sq_head;
        for ( int j=0; j<cur_Sq_size; j++){
#ifdef MAX_FRAG_16
        	if ( l_queue->index <= mh->mcexor_ack_hdr.pID ){
#endif
#ifdef MAX_FRAG_1
          	if ( l_queue->index == mh->mcexor_ack_hdr.pID ){
#endif
    			l_queue->flag = ACKED;
    		}
    		l_queue = l_queue->next;
    	}
    }

}

// free memory used by Sq and NS pkts
void
MacMCExOR::clear_ACKED_in_Sq( )
{
        struct mac_queue       * l_queue = NULL;

        if ( cur_Sq_size == 0 )
        {
          return;
        }else if ( cur_Sq_size == 1 ) {
			l_queue = Sq_head;
			if ( l_queue->flag == ACKED ) {
				Packet::free(Sq_head->NSpkt);
	        	Sq_head->NSpkt = NULL;
		  	    free(Sq_head);
				Sq_head = NULL;
				Sq_tail = NULL;
				cur_Sq_size = 0;
			}
        }else {
            while ( Sq_head != NULL && Sq_head->flag == ACKED ) {
                remove_Sq_head();
            }
        }
        return;
}

// free memory used by Sq and NS pkts
void
MacMCExOR::clear_failed_in_Sq( int howmany )
{
//printf("MacMCExOR::clear_failed_in_Sq, cur_Sq_size=%d, in=%d\n", cur_Sq_size, index_);

		if ( cur_Sq_size == 0 )
        {
	        return;
        }else if ( cur_Sq_size == 1 ) {
        	Packet::free(Sq_head->NSpkt);
        	Sq_head->NSpkt = NULL;
			free(Sq_head);
			Sq_head = NULL;
			Sq_tail = NULL;
			cur_Sq_size = 0;
        }else {
        	int i=0;
            while ( Sq_head != NULL && i<howmany ) {
                remove_Sq_head();
                i ++;
//printf("MacMCExOR::clear_failed_in_Sq, cur_Sq_size=%d, in=%d\n", cur_Sq_size, index_);
            }
        }
        return;
}


// remove all successfully transmitted pkts in the Rq
void
MacMCExOR::clear_repeated_MAC_ACKs_in_Rq( )
{
    struct mac_queue       * l_queue = NULL;

    if ( cur_Rq_size == 0 )
    {
      return;
    }else if ( cur_Rq_size == 1 )
    {
                l_queue = Rq_head;
                if ( l_queue->flag == RIGHT && l_queue->index == -1 )
                {
                        free(Rq_head);
                        Rq_head = NULL;
                        Rq_tail = NULL;
                        cur_Rq_size = 0;
                }
    }else {
        while ( Rq_head != NULL && l_queue->flag == RIGHT && l_queue->index == -1  )
        {
            remove_Rq_head();
            cur_Rq_size --;
        }
    }
    return;
}

void
MacMCExOR::print_mac_queue(char *fname, struct mac_queue *q)
{
    struct mac_queue           * l_queue;
    struct hdr_cmn             * l_ch;
    struct mcexor_mac_data_hdr * mh;

    if ( q == NULL ) {
        printf("This queue is empty! in=%d\n", index_);
        return;
    }

    l_queue = q;
    if ( q==Rq_head )
        fprintf(stdout,
            "%s --- (INDEX: %d, time: %2.9f) --- Rq\n",
            fname, index_, Scheduler::instance().clock());

    if ( q==Sq_head )
        fprintf(stdout,
            "%s --- (INDEX: %d, time: %2.9f) --- Sq\n",
            fname, index_, Scheduler::instance().clock());

    while ( l_queue != NULL ) {
        l_ch = HDR_CMN(l_queue->NSpkt);
        mh   = HDR_MAC_MCEXOR_DATA(l_queue->NSpkt);
        fprintf(stdout, "\t uid/flag/s/d/size: %d %d %d %d %d\n",
        		l_ch->uid(), l_queue->flag, ETHER_ADDR(mh->dh_ta), ETHER_ADDR(mh->dh_ra), l_ch->size());
        l_queue = l_queue->next;
    }

    return;
}




int
MacMCExOR::hops_to_dst(Packet *p)
{
    struct hdr_cmn *ch   = HDR_CMN(p);
    int        temp      = -1;



    for ( int i=MAX_FORWARDERS-1; i>=0; i--) {
        if ( ch->f_list[i] >= 0 ) {
            temp ++;
        }
        if ( ch->f_list[i] == index_ ) {
            return temp;
        }
    }

    printf("MacMCExOR::hops_to_dst, should not reach here! in=%d\n", index_);
    trace_pkt("BER 22222222222", p);
    exit(0);
}

int
MacMCExOR::hops_to_dst_ack(Packet *p)
{
    struct hdr_cmn  *ch     = HDR_CMN(p);
    int             temp    = -1;


    for ( int i=MAX_FORWARDERS-1; i>=0; i--) {
        if ( ch->f_list[i] >= 0 ) {
            temp ++;
        }
        if ( ch->f_list[i] == index_ ) {
            return temp;
        }
    }
    printf("MacMCExOR::hops_to_dst_ack, should not reach here! in=%d\n", index_);
    exit(0);
}

int
MacMCExOR::hops_to_src_ack(Packet *p)
{
    struct hdr_cmn  *ch     = HDR_CMN(p);
    int             temp    = -1;


    for ( int i=0; i<MAX_FORWARDERS; i++) {
        if ( ch->f_list[i] >= 0 ) {
            temp ++;
        }
        if ( ch->f_list[i] == index_ ) {
            return temp;
        }
    }
    printf("MacMCExOR::hops_to_src_ack, should not reach here! in=%d\n", index_);
    exit(0);
}
