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
 * $Header: /tianji.cvsroot/newripple/ns-2.30/mac/ripple/mac-ripple.cc,v 1.9 2009/01/28 15:20:17 ripple Exp $
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

// #define TIANJI_DEBUG_8
// #define TIANJI_DEBUG_VERY_SHORT
// #define TIANJI_DEBUG_SHORT
#define TIANJI_NEW_DEBUG

#include "arp.h"
#include "ll.h"
#include "stdio.h"
#include "stdlib.h"
#include "mac-timers-ripple.h"
#include "mac-ripple.h"
#include "cmu-trace.h"

// Added by Sushmita to support event tracing
#include "agent.h"
#include "basetrace.h"


int temp_seq_mac_ack = -1;


#ifdef TIANJI_RIPPLE
int ripple_mac_data_hdr::offset_;
static class MacRippleDataHeaderClass : public PacketHeaderClass {
public:
        MacRippleDataHeaderClass() : PacketHeaderClass("PacketHeader/MacRippleData",
                                             sizeof(ripple_mac_data_hdr)) {
                bind_offset(&ripple_mac_data_hdr::offset_);
        }
} class_ripple_mac_data_hdr;

int ripple_mac_ack_hdr::offset_;
static class MacRippleAckHeaderClass : public PacketHeaderClass {
public:
        MacRippleAckHeaderClass() : PacketHeaderClass("PacketHeader/MacRippleAck",
                                             sizeof(ripple_mac_ack_hdr)) {
                bind_offset(&ripple_mac_ack_hdr::offset_);
        }
} class_ripple_mac_ack_hdr;
#endif



inline void
MacRipple::checkBackoffTimer()
{
	if(is_idle() && mhBackoff_.paused())
		mhBackoff_.resume(phymib_.getDIFS());
	if(! is_idle() && mhBackoff_.busy() && ! mhBackoff_.paused())
		mhBackoff_.pause();
}

inline void
MacRipple::transmit(Packet *p, double timeout)
{
	struct hdr_cmn             *ch       = HDR_CMN(p);
	struct ripple_mac_data_hdr *ddh      = RIPPLE_MAC_DATA_HDR(p);
	struct ripple_mac_ack_hdr  *dah      = RIPPLE_MAC_ACK_HDR(p);
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

if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
	src      = ETHER_ADDR(ddh->dh_ta);
	dst      = ETHER_ADDR(ddh->dh_ra);
	// get the pointer of pkts in this frame
	l_data = (PacketData *)p->userdata();
	l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
	l_NSpkt = l_data_p->NSpkt;
	printf("\n\n\nxmit in=%d, n=%d, puid=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ddh->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);
}
if ( ch->ptype() == PT_MAC_RIPPLE_ACK ) {
	src      = ETHER_ADDR(dah->dh_ta);
	dst      = ETHER_ADDR(dah->dh_ra);
	printf("\n\n\nxmit in=%d, n=%d, type=%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtill=%lf, to=%lf\n",
			index_, ch->num_pkts(), dah->dh_fc.fc_type, src, dst, ch->size(),
			ch->error(), Scheduler::instance().clock(), Scheduler::instance().clock()+txtime(p), Scheduler::instance().clock()+timeout);
}


 trace_pkt("xmit",p);
#endif

    p_copy_for_tx = p->copy();
	downtarget_->recv(p_copy_for_tx, this);

	mhSend_.start(timeout);
	mhIF_.start(txtime(p));
}
inline void
MacRipple::setRxState(MacState newState)
{
	rx_state_ = newState;
	checkBackoffTimer();
}

inline void
MacRipple::setTxState(MacState newState)
{
	tx_state_ = newState;
	checkBackoffTimer();
}


/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class MacRippleClass : public TclClass {
public:
	MacRippleClass() : TclClass("Mac/Ripple") {}
	TclObject* create(int, const char*const*) {
	return (new MacRipple());

}
} class_mac_ripple;


/* ======================================================================
   Mac  and Phy MIB Class Functions
   ====================================================================== */

PHY_MIB::PHY_MIB(MacRipple *parent)
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

MAC_MIB::MAC_MIB(MacRipple *parent)
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
MacRipple::MacRipple() :
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
	tcl.evalf("Mac/Ripple set basicRate_");
	if (strcmp(tcl.result(), "0") != 0)
		bind_bw("basicRate_", &basicRate_);
	else
		basicRate_ = bandwidth_;

	tcl.evalf("Mac/Ripple set dataRate_");
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

	data_daemon = construct_data_daemon();
	ack_daemon  = construct_ack_daemon();
}


int
MacRipple::command(int argc, const char*const* argv)
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
void MacRipple::trace_event(char *eventtype, Packet *p)
{
        if (et_ == NULL) return;
        char *wrk = et_->buffer();
        char *nwrk = et_->nbuffer();


        struct ripple_mac_data_hdr* dh = RIPPLE_MAC_DATA_HDR(p);

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
MacRipple::trace_pkt(char *fname, Packet *p)
{
	struct hdr_cmn                 * ch        = HDR_CMN(p);
	struct ripple_mac_data_hdr     * ddh       = RIPPLE_MAC_DATA_HDR(p);
	struct ripple_mac_ack_hdr      * dah       = RIPPLE_MAC_ACK_HDR(p);
	PacketData                     * l_data;
	struct data_part               * l_data_p;
	Packet                         * l_NSpkt   = NULL;
	struct hdr_cmn                 * l_ch;
	int                              src;
	int                              dst;
	int                              temp_dur;

	if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
		src      = ETHER_ADDR(ddh->dh_ta);
		dst      = ETHER_ADDR(ddh->dh_ra);
		temp_dur = ddh->dh_duration;
	}
	if ( ch->ptype() == PT_MAC_RIPPLE_ACK ) {
		src      = ETHER_ADDR(dah->dh_ta);
		dst      = ETHER_ADDR(dah->dh_ra);
		temp_dur = dah->dh_duration;
    }


	fprintf(stdout, "%s --- (INDEX: %d, time: %2.9f, addr=%p)\n", fname, index_, Scheduler::instance().clock(), p);
	fprintf(stdout, "\t dur/src/dst/type/size/n:%d %d %d %s %d %d\n",
			 temp_dur, src, dst, packet_info.name(ch->ptype()), ch->size(), ch->num_pkts());
	fprintf(stdout, "\t f.list: %d %d %d %d %d %d\n",
			ch->f_list[0], ch->f_list[1], ch->f_list[2],
			ch->f_list[3], ch->f_list[4], ch->f_list[5]);
	if ( ch->ptype() == PT_MAC_RIPPLE_ACK )
	 fprintf(stdout, "\t ack_bitmap: %d %d %d %d %d %d\n",
			 dah->ripple_ack_hdr.ack_bitmap[0],
			 dah->ripple_ack_hdr.ack_bitmap[1],
			 dah->ripple_ack_hdr.ack_bitmap[2],
			 dah->ripple_ack_hdr.ack_bitmap[3],
			 dah->ripple_ack_hdr.ack_bitmap[4],
			 dah->ripple_ack_hdr.ack_bitmap[5]);

	l_data = (PacketData *)p->userdata();
	for ( int i=0; i<ch->num_pkts(); i++) {
			l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
			l_NSpkt  = l_data_p->NSpkt;
			l_ch     = HDR_CMN(l_NSpkt);
			fprintf(stdout, "\t uid/es/ed/size:%d %d %d %d \n", l_ch->uid(), src, dst, l_ch->size() );
	}

}

void
MacRipple::dump(char *fname)
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
		"\tDefer: %d, Backoff: %d (%d, exp %f), Recv: %d, Send: %d If: %d Nav: %d \n",
		mhDefer_.busy(), mhBackoff_.busy(), mhBackoff_.paused(), mhBackoff_.expire(),
		mhRecv_.busy(), mhSend_.busy(), mhIF_.busy(), mhNav_.busy());
}


/* ======================================================================
   Packet Headers Routines
   ====================================================================== */
inline int
MacRipple::hdr_dst(char* hdr, int dst )
{
	struct ripple_mac_data_hdr *dh = (struct ripple_mac_data_hdr *) hdr;

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
MacRipple::hdr_src(char* hdr, int src )
{
	struct ripple_mac_data_hdr *dh = (struct ripple_mac_data_hdr*) hdr;
        if(src > -2)
               STORE4BYTE(&src, (dh->dh_ta));
        return ETHER_ADDR(dh->dh_ta);
}

inline int
MacRipple::hdr_type(char* hdr, u_int16_t type)
{
	struct ripple_mac_data_hdr *dh = (struct ripple_mac_data_hdr*) hdr;
	if(type)
		STORE2BYTE(&type,(dh->dh_body));
	return GET2BYTE(dh->dh_body);
}


/* ======================================================================
   Misc Routines
   ====================================================================== */
inline int
MacRipple::is_idle()
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
MacRipple::discard(Packet *p, const char* why)
{
	ripple_mac_data_hdr* mh = RIPPLE_MAC_DATA_HDR(p);
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
MacRipple::capture(Packet *p)
{
	/*
	 * Update the NAV so that this does not screw
	 * up carrier sense.
	 */
	set_nav(usec(phymib_.getEIFS() + txtime(p)));
}

void
MacRipple::collision(Packet *p)
{
printf("collision, can this happen?? in=%d\n", index_);
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
		}
		break;
	default:
		assert(0);
	}
}

void
MacRipple::tx_resume(int priority)
{
	double rTime;
	assert(mhSend_.busy() == 0);
	assert(mhDefer_.busy() == 0);

//dump("tx_resume dump");
	if(pktCTRL_) {
		mhDefer_.start((double)priority*phymib_.getSIFS());
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
			struct ripple_mac_data_hdr *mh = RIPPLE_MAC_DATA_HDR(pktTx_);

			if ((u_int32_t) ch->size() < macmib_.getRTSThreshold() || ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
				if (bugFix_timer_)
					mhBackoff_.start(cw_, is_idle(),  phymib_.getDIFS());
				else {
					rTime = (Random::random() % cw_) * phymib_.getSlotTime();
					mhDefer_.start(phymib_.getDIFS() +   rTime);
				}
            } else  mhDefer_.start(phymib_.getSIFS());
		}
	} else if(callback_) {
		Handler *h = callback_;
		callback_ = 0;
		h->handle((Event*) 0);
	}
	setTxState(MAC_IDLE);
}

void
MacRipple::rx_resume()
{
	assert(pktRx_ == 0);
	assert(mhRecv_.busy() == 0);
	setRxState(MAC_IDLE);
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
MacRipple::backoffHandler()
{
#ifdef TIANJI_NEW_DEBUG
printf("MacRipple::backoffHandler, %d\n", index_);
if ( pktTx_ && index_==0 ) trace_pkt("trace pktTx_ backoffHandler", pktTx_);
#endif
	if(pktCTRL_) {
		assert(mhSend_.busy() || mhDefer_.busy());
		return;
	}

	if(check_pktRTS() == 0)
		return;

	if(check_pktTx() == 0)
		return;
}

void
MacRipple::deferHandler()
{
#ifdef TIANJI_DEBUG_8
printf("MacRipple::deferHandler, now=%lf, %d\n", Scheduler::instance().clock(),index_);
#endif
	assert(pktCTRL_ || pktRTS_ || pktTx_);

	if(check_pktCTRL() == 0)
		return;
	assert(mhBackoff_.busy() == 0);
	if(check_pktRTS() == 0)
		return;
	if(check_pktTx() == 0)
		return;
}

void
MacRipple::navHandler()
{
#ifdef TIANJI_DEBUG_8
printf("MacRipple::navHandler, %d\n", index_);
#endif
	if(is_idle() && mhBackoff_.paused())
		mhBackoff_.resume(phymib_.getDIFS());
}

void
MacRipple::recvHandler()
{
	recv_timer();
}

void
MacRipple::sendHandler()
{
	send_timer();
}


void
MacRipple::txHandler()
{
	if (EOTtarget_) {
		assert(eotPacket_);
		EOTtarget_->recv(eotPacket_, (Handler *) 0);
		eotPacket_ = NULL;
	}
	tx_active_ = 0;
#ifdef TIANJI_NEW_DEBUG
printf("MacRipple::txHandler, in=%d\n", index_);
if ( index_ == 0 && pktTx_ )
	trace_pkt("trace pktTx_ txHandler",pktTx_);
#endif
}


/* ======================================================================
   The "real" Timer Handler Routines
   ====================================================================== */
void
MacRipple::send_timer()
{
#ifdef TIANJI_NEW_DEBUG
printf("send_timer, %d\n", index_);
dump("send_timer");
if ( index_ == 0 && pktTx_ ) trace_pkt("trace pkt send_timer", pktTx_);
#endif

//    struct ripple_mac_data_hdr *ddh         = RIPPLE_MAC_DATA_HDR(pktTx_);

    switch(tx_state_) {
		case MAC_CTS:
			assert(pktCTRL_);
			pktCTRL_ = 0;
			break;
		case MAC_SEND:
			if ( rx_state_ != MAC_IDLE  ) {
				printf("RetransmitData, what is going on here?\n");
				exit(0);
			}

			// src retx
			if ( pktTx_ && ETHER_ADDR(RIPPLE_MAC_DATA_HDR(pktTx_)->dh_ta) == index_ )
				RetransmitDATA();
			else {
				// forwarders do not re tx
#ifdef TIANJI_NEW_DEBUG
    printf("send_timer, forwaders do not retx, free data_daemon now, in=%d\n", index_);
#endif
	            data_daemon = reconstruct_data_daemon(); // free NS pkts in it too
	            pktTx_ = 0;
	            setTxState(MAC_IDLE);
			}

			break;
		/*
		 * Sent an ACK, and now ready to resume transmission.
		 */
		case MAC_ACK:
			assert(pktCTRL_);
#ifdef TIANJI_NEW_DEBUG
    printf("send_timer, Sent an pktCTRL_, and now ready to resume transmission, in=%d\n", index_);
#endif

			if ( pktCTRL_ ) {
				if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_DATA ) {
					// renew data if not empty
					data_daemon = reconstruct_data_daemon(); // free NS pkts in it too
					pktTx_ = 0;
				}else if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_ACK ) {
					// renew ack if not empty
					Packet::free(ack_daemon);
					ack_daemon = construct_ack_daemon();
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
//if ( index_ == 0 && pktTx_ ) trace_pkt("trace pkt send_timer", pktTx_);
#endif

	tx_resume(-1);
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
MacRipple::check_pktCTRL()
{
	struct hdr_cmn             * ch       = HDR_CMN(pktCTRL_);
	double                       timeout;
    int                          temp;
    struct ripple_mac_data_hdr * mdh      = RIPPLE_MAC_DATA_HDR(pktCTRL_);
	struct ripple_mac_ack_hdr  * mah      = RIPPLE_MAC_ACK_HDR(pktCTRL_);
	struct frame_control         temp_fc;
    int                          temp_dur;

    if( pktCTRL_ == 0 )	                               return -1;
	if(tx_state_ == MAC_CTS || tx_state_ == MAC_ACK)   return -1;

	if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
		// there may be more pkts available during backoff, pktCTRL_ == data_daemon before calling
		update_data_daemon_to_send( RIGHT );

		temp_fc  = mdh->dh_fc;
		temp_dur = mdh->dh_duration;
        temp     = hops_to_dst(pktCTRL_);
	}
	if ( ch->ptype() == PT_MAC_RIPPLE_ACK ) {
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
    	// RIPPLE always uses MAC_Subtype_Data for both MAC data and ACK frames
		setTxState(MAC_ACK);
		timeout = temp * (DSSS_MaxPropagationDelay + phymib_.getSIFS()) + sec(temp_dur);
		break;
	default:
		fprintf(stderr, "check_pktCTRL:Invalid MAC Control subtype\n");
		exit(1);
	}


	transmit(pktCTRL_, timeout);
	return 0;
}

int
MacRipple::check_pktRTS()
{
	struct ripple_mac_data_hdr *mh;
	double timeout;

	assert(mhBackoff_.busy() == 0);

	if(pktRTS_ == 0)
 		return -1;
	mh = RIPPLE_MAC_DATA_HDR(pktRTS_);

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
MacRipple::check_pktTx()
{
	struct ripple_mac_data_hdr * mh;
	double                       timeout;
    int                          temp;

	assert(mhBackoff_.busy() == 0);

// there may be more pkts available during backoff
	update_data_daemon_to_send( WRONG );
	if ( HDR_CMN(data_daemon)->num_pkts() > 0 )
		pktTx_ = data_daemon;
	else
		pktTx_ = 0;

	if(pktTx_ == 0)
		return -1;

	mh = RIPPLE_MAC_DATA_HDR(pktTx_);

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
                        timeout = temp*DSSS_MaxPropagationDelay
                                + temp*phymib_.getSIFS()
                                + sec(mh->dh_duration);
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
MacRipple::sendDATA(Packet *p)
{
	assert(pktTx_ == 0);

        if ( data_daemon!=NULL )
        {
                pktTx_ = data_daemon;
        }else
        {
                printf("MacRipple::sendDATA, no frame to send, quit...%d\n",index_);
                exit(0);
        }

}


void
MacRipple::RetransmitDATA()
{
	struct hdr_cmn *ch;
	struct ripple_mac_data_hdr *mh;
	u_int32_t *rcount, thresh;
	assert(mhBackoff_.busy() == 0);

	assert(pktTx_);
	assert(pktRTS_ == 0);

printf("Retx data, in=%d. \n", index_);


	ch = HDR_CMN(pktTx_);
	mh = RIPPLE_MAC_DATA_HDR(pktTx_);
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

	if(*rcount >= thresh) {
		/* IEEE Spec section 9.2.3.5 says this should be greater than
		   or equal */
		macmib_.FailedCount++;
		/* tell the callback the send operation failed
		   before discarding the packet */

		*rcount = 0;

printf("Retx data, retry out??\n");
Packet::free(pktTx_);
exit(0);
// if collisions retrylimit times, do not do anything, just keep trying!!!
		rst_cw();
	} else {

                // form a new frame, as more pkts may be available
                if ( src == index_ )  {// not forwarding
                    if ( cur_Sq_size > 0  )  update_data_daemon_to_send( WRONG  );
                    else goto exit_retx; // do not retx MAC ACKs
                } else if ( src < index_ ) { // forwarding
                    if ( cur_correct_in_Rq > 0 ) update_data_daemon_to_send( RIGHT );
                    else goto exit_retx;  // should not happen?
                } else goto exit_retx; // do not retx MAC ACKs

		mh->dh_fc.fc_retry = 1;

		inc_cw();
		mhBackoff_.start(cw_, is_idle());
	}
        return;

exit_retx:
        pktTx_ = 0;
        rst_cw();
        mhBackoff_.start(cw_, is_idle());
        return;

}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
MacRipple::send(Packet *p, Handler *h)
{
	double rTime;
	struct ripple_mac_data_hdr* dh = RIPPLE_MAC_DATA_HDR(p);

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
			 * the backoff timer.
			 */
			mhBackoff_.start(cw_, is_idle());
		}
	}
}

void
MacRipple::recv(Packet *p, Handler *h)
{
	struct hdr_cmn             *hdr   = HDR_CMN(p);
	struct ripple_mac_data_hdr *mdh   = RIPPLE_MAC_DATA_HDR(p);
	struct ripple_mac_ack_hdr  *mah   = RIPPLE_MAC_ACK_HDR(p);
	int                         dst;
	int                         src;

	if(hdr->direction() == hdr_cmn::DOWN) {
		hdr->ptype() = PT_MAC_RIPPLE_DATA;
		dst = ETHER_ADDR(mdh->dh_ra);
		src = ETHER_ADDR(mdh->dh_ta);
#ifdef TIANJI_NEW_DEBUG
printf("\nRecv down, now=%lf, forwarders=%d %d %d %d %d %d, size=%d, uid=%d, in=%d\n",
		Scheduler::instance().clock(), hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4], hdr->f_list[5], hdr->size(), hdr->uid(), index_);
#endif

		if ( cur_Sq_size < MAX_Sq_SIZE ) {
			 push_Sq(p); // drain the ifq and save in Sq
		} else {
			 Packet::free(p);
		}

		daemon_handler = h;
		callback_      = h;

		update_data_daemon_to_send(WRONG);
		send(data_daemon, daemon_handler);
#ifdef TIANJI_NEW_DEBUG
		if ( (index_ == 0 && pktTx_) || (index_ == 3 && pktTx_) ) {
			trace_pkt("trace pktTx_ recv down",pktTx_);
			print_mac_queue("Sq recv down", Sq_head);
		}
#endif
		if ( cur_Sq_size < MAX_Sq_SIZE ) {
			// fetch another pkt if any from the upper layer
			Handler *h_temp = h;
			h = 0;
			h_temp->handle((Event*) 0);
		}
		return;
	}

//#ifdef TIANJI_NEW_DEBUG
//	if ( index_ == 0 && pktTx_ ) {
//		trace_pkt("trace pktTx_ recv up",pktTx_);
//		print_mac_queue("Sq recv up", Sq_head);
//
//	}
//#endif


	if ( hdr->ptype() == PT_MAC_RIPPLE_DATA ) {
		dst = ETHER_ADDR(mdh->dh_ra);
		src = ETHER_ADDR(mdh->dh_ta);
	}
	if ( hdr->ptype() == PT_MAC_RIPPLE_ACK ) {
		dst = ETHER_ADDR(mah->dh_ra);
		src = ETHER_ADDR(mah->dh_ta);
	}



	if( tx_active_ && hdr->error() == 0) {
		hdr->error() = 1;
	}

	if( am_i_in_f_list(p) == WRONG ) {
		hdr->error() = 1;
	}


	// forwarders get an ACK, should update its frame according to bitmap, because
	// the bitmap may contain acks for myself, i.e., I may have piggybacked some packets
	if ( am_i_forwarder(p)==RIGHT && hdr->ptype() == PT_MAC_RIPPLE_ACK && pktCTRL_ && mhDefer_.busy() ) {
		// 1. stop waiting to forward, as the incoming ACK means that the data frame has been
		// received by the destination.
  	    mhDefer_.stop();
		// 2. check if this incoming bitmap can acknowledge packets for myself, which will be done in recvData
		// 3. free existing frame memory
  	    if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_DATA ){
   	    	data_daemon = reconstruct_data_daemon();
   	    	pktCTRL_ = 0;
  	    }else if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_ACK ){
   	    	printf("recv up, can this happen? waiting to send a MAC ACK, another one is coming?? in=%d\n", index_);
  	    }
  	}

    // forwarders should cancel their tx if hear high priority tx
	if (  am_i_in_f_list(p) && hear_a_higher_pri_tx(p) == RIGHT ) {
		if( mhDefer_.busy() )	mhDefer_.stop();
		if ( pktCTRL_ ) {
	  	    if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_DATA ){
	   	    	data_daemon = reconstruct_data_daemon();
	   	    	pktCTRL_ = 0;
	  	    }else if ( HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_ACK ){
	   	    	ack_daemon = construct_ack_daemon();
	   	    	pktCTRL_ = 0;
	  	    }
		}
		hdr->error() = 1; //should not receive this incoming pkt
	}


	if ( src > num_sta || dst > num_sta ) { // htp from 802.11
		hdr->error() = 1;
	}

#ifdef TIANJI_NEW_DEBUG
if ( hdr->ptype() == PT_MAC_RIPPLE_DATA )
    printf("Recv up data, forwarders=%d %d %d %d %d %d, size=%d, uid=%d, type=%d, err=%d, in=%d\n",
    		hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4],
    		hdr->f_list[5], hdr->size(), hdr->uid(), mdh->dh_fc.fc_type, hdr->error(), index_);
if ( hdr->ptype() == PT_MAC_RIPPLE_ACK )
    printf("Recv up ack, forwarders=%d %d %d %d %d %d, size=%d, uid=%d, type=%d, err=%d,  in=%d\n",
    		hdr->f_list[0], hdr->f_list[1], hdr->f_list[2], hdr->f_list[3], hdr->f_list[4],
    		hdr->f_list[5], hdr->size(), hdr->uid(), mah->dh_fc.fc_type, hdr->error(),  index_);
//dump("recv up before");
//trace_pkt("recv up before, p", p);
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

#ifdef TIANJI_NEW_DEBUG
//	dump("recv up after");
#endif
}

void
MacRipple::recv_timer()
{
	hdr_cmn             *ch     = HDR_CMN(pktRx_);
	ripple_mac_data_hdr *mdh    = RIPPLE_MAC_DATA_HDR(pktRx_);
	ripple_mac_ack_hdr  *mah    = RIPPLE_MAC_ACK_HDR(pktRx_);
	u_int8_t            type    ;
	u_int8_t            subtype ;
	int                 dst, src;

	if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
		type    = mdh->dh_fc.fc_type;
		subtype = mdh->dh_fc.fc_subtype;
		dst     = ETHER_ADDR(mdh->dh_ra);
		src     = ETHER_ADDR(mdh->dh_ta);
	}
    if ( ch->ptype() == PT_MAC_RIPPLE_ACK )  {
    	type    = mah->dh_fc.fc_type;
    	subtype = mah->dh_fc.fc_subtype;
    	dst     = ETHER_ADDR(mah->dh_ra);
    	src     = ETHER_ADDR(mah->dh_ta);
    }

printf("MacRipple::recv_timer, type=%d, subtype=%d, now=%lf, id=%d, in=%d\n",
		type, subtype, Scheduler::instance().clock(), ch->uid(), index_);
//if (index_==0 && pktTx_ ) {
//	dump("recv_timer");
//	trace_pkt("trace pktTx recv_timer", pktTx_);
//}

	assert(pktRx_);
	assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);

	if(tx_active_) {
    	free_ripple_pkt(pktRx_);
    	goto done;
	}

	if(rx_state_ == MAC_COLL) {
		set_nav(usec(phymib_.getEIFS()));
		free_ripple_pkt(pktRx_);
		goto done;
	}

	if( ch->error() == 1 ) {
		set_nav(usec(phymib_.getEIFS()));
		free_ripple_pkt(pktRx_);
		goto done;
	}

// src/dst are also forwarders in this function
    if ( am_i_in_f_list(pktRx_) == WRONG ) {
    	if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) set_nav(mdh->dh_duration);
    	if ( ch->ptype() == PT_MAC_RIPPLE_ACK ) set_nav(mah->dh_duration);
    	free_ripple_pkt(pktRx_);
    	goto done;
    }


	switch(type) {
	case MAC_Type_Management:
		Packet::free(pktRx_);
		goto done;
	case MAC_Type_Control:
		switch(subtype) {
		case MAC_Subtype_RTS:
			printf("Ripple does not support RTS, quit!\n");
			exit(0);
		case MAC_Subtype_CTS:
            printf("Ripple does not support CTS, quit!\n");
            exit(0);
		case MAC_Subtype_ACK:
            printf("Ripple does not support 802.11 MAC ACK, quit!\n");
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
 done:
	pktRx_ = 0;
	rx_resume();
}


/*
 * txtime()	- pluck the precomputed tx time from the packet header
 */
double
MacRipple::txtime(Packet *p)
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
MacRipple::txtime(double psz, double drt)
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
MacRipple::recvDATA(Packet *p)
{
/*************************************************************
 *
 * Called by forwarders and destination
 * Input: p can be MAC data or MAC ACK format
 *
 *************************************************************/

        struct           hdr_cmn   *ch          = HDR_CMN(p);
        int                         pkttype     = ch->ptype();
        struct ripple_mac_data_hdr *ddh         = RIPPLE_MAC_DATA_HDR(p);
        struct ripple_mac_ack_hdr  *dah         = RIPPLE_MAC_ACK_HDR(p);
        Packet                     *l_NSpkt     = NULL;
        int                        dst, src, size, priority, carried_num_pkt;
        double                     per;
        double                     var;
        PacketData                 *l_data;
        struct data_part           *l_data_p;

        if ( pkttype == PT_MAC_RIPPLE_DATA ) {
        	dst = ETHER_ADDR(ddh->dh_ra);
            src = ETHER_ADDR(ddh->dh_ta);
        }
        if ( pkttype == PT_MAC_RIPPLE_ACK ) {
        	dst = ETHER_ADDR(dah->dh_ra);
        	src = ETHER_ADDR(dah->dh_ta);
        }

        size            = ch->size();
        carried_num_pkt = ch->num_pkts();

        if ( pkttype == PT_MAC_RIPPLE_DATA && carried_num_pkt <=0  ){
            printf("recvDATA, carried_num_pkt should not < 0 for data frames, quit! in=%d \n", index_);
            exit(0);
        }

        // 1. Dst gets a RIPPLE data
        if ( dst == index_ && pkttype == PT_MAC_RIPPLE_DATA ) {
#ifdef TIANJI_NEW_DEBUG
	printf("recvData, dst gets a data\n");
#endif
            l_data = (PacketData *)p->userdata();
            for ( int i=0; i<carried_num_pkt; i++) {
                l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
                l_NSpkt = l_data_p->NSpkt;

                per = 1 - pow ( 1-BER, size*8 );
                var = Random::uniform(0.0,1.0);
                per = -0.1;
                if (var <= per) // corrupted
                        push_Rq(l_NSpkt, WRONG, TAIL, i);
                else
                        push_Rq(l_NSpkt, RIGHT, TAIL, i);
            }

            if ( cur_correct_in_Rq > 0 ) {
                 // send MAC ACK after a SIFS
                 update_ack_daemon_to_send( WRONG, p );
                 pktCTRL_ = ack_daemon;
                 priority = 1;
            }  else {
                 printf("This should not happen, quit!\n");
                 exit(0);
            }
            // pass pkts to upper layer
            recv_correct_pkts( uptarget_ );
        }

        // 2. Src gets a RIPPLE ACK, updates Sq according to the ACK bitmap
        if ( dst == index_ && pkttype == PT_MAC_RIPPLE_ACK) {
#ifdef TIANJI_NEW_DEBUG
	printf("recvData, src gets a MAC ACK, in=%d\n", index_);
#endif
            update_flags_in_Sq(p);
            clear_ACKED_in_Sq();

            mhSend_.stop();

			data_daemon = reconstruct_data_daemon();
		    pktTx_ = 0;

            if ( mhBackoff_.busy() )   mhBackoff_.stop();

            rst_cw();
        }

        // 3. Forwarder gets a RIPPLE data
        if ( am_i_forwarder(p)==RIGHT && pkttype == PT_MAC_RIPPLE_DATA ) {
#ifdef TIANJI_NEW_DEBUG
	printf("recvData, forwarder gets a data, in=%d\n", index_);
#endif

            data_daemon = p->copy();  // copy received info from p into data_daemon
            update_data_daemon_to_send( RIGHT );
            // need to use pktCTRL_ to forward received data, so that SIFS level timing is possible
            pktCTRL_ = data_daemon;
            priority = my_priority(p);
        }

        // 4. Forwarder gets a RIPPLE ACK, updates Sq according to the ACK bitmap
        if ( am_i_forwarder(p)==RIGHT && pkttype == PT_MAC_RIPPLE_ACK ) {
#ifdef TIANJI_NEW_DEBUG
	printf("recvData, forwarder gets a MAC ACK, in=%d\n", index_);
#endif

            update_flags_in_Sq(p);
            clear_ACKED_in_Sq();

            if ( pktCTRL_ &&  HDR_CMN(pktCTRL_)->ptype() == PT_MAC_RIPPLE_DATA ){
            	// free previous data packet, does this always happen?
#ifdef TIANJI_NEW_DEBUG
	printf("recvData, forwarder gets a MAC ACK, free previous data packet, does this always happen? \n");
#endif
                data_daemon = reconstruct_data_daemon();
                pktCTRL_ = 0;
            }

            // forward the received mac ack
            ack_daemon = p->copy();  // copy received info from p into data daemon
            update_ack_daemon_to_send( RIGHT, p );
            pktCTRL_ = ack_daemon;
            priority = my_priority(p);

        }


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
				// Ripple goes here!!!
				tx_resume(priority);
			}
		}

done:

		// free used memory
		// 1. Dst gets a RIPPLE data, free p, but do not free pkts in it
		if ( dst == index_ && pkttype == PT_MAC_RIPPLE_DATA ) {
			Packet::free(p);
		}

		// 2. Src gets a RIPPLE ACK, free p
		if ( dst == index_ && pkttype == PT_MAC_RIPPLE_ACK) {
			Packet::free(p);
		}

		// 3. Forwarder gets a RIPPLE data, free p and pkts in it
		if ( am_i_forwarder(p)==RIGHT && pkttype == PT_MAC_RIPPLE_DATA ) {
			free_ripple_pkt(p);
		}

		// 4. Forwarder gets a RIPPLE ACK
		if ( am_i_forwarder(p)==RIGHT && pkttype == PT_MAC_RIPPLE_ACK ) {
			Packet::free(p);
		}
}






/**********************************************************************
 *
 *  Ripple scheme: Tianji Li, Jul 2008, Hamilton Institute, NUIM, Ireland
 *
 *
 **********************************************************************/


int
MacRipple::pkt_e2esrc(Packet *p)
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
MacRipple::pkt_e2edst(Packet *p)
{
        struct hdr_cmn *ch           = HDR_CMN(p);
        for ( int i=MAX_FORWARDERS-1; i>0; i--) {
            if ( ch->f_list[i] >= 0 ) {
                return ch->f_list[i];
            }
        }
        printf("MacRippple pkt_e2edst, this should not happen!\n");
        exit(0);
}


void
MacRipple::push_Sq(Packet *p)
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
MacRipple::push_Rq(Packet *p, int flag, int head_or_tail, int forwarding)
{
        hdr_cmn                    * ch          = HDR_CMN(p);
        int                          founded     = WRONG;
        struct mac_queue           * l_queue;
        struct mac_queue           * temp_queue;

        if ( head_or_tail == HEAD ) {
            // this pkt may have already queued
            if ( Rq_head != NULL && flag == RIGHT ) {
                temp_queue = Rq_head;
                while ( temp_queue != NULL ) {
                    if ( temp_queue->index == ch->uid() ) {
                        if ( temp_queue->flag == WRONG ) temp_queue->flag = RIGHT;
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
            l_queue->e2esrc   = pkt_e2esrc(p);
            l_queue->e2edst   = pkt_e2edst(p);


            if ( cur_Rq_size == 0 ) {
                Rq_head = l_queue;
                Rq_tail = l_queue;
            } else if ( forwarding == 0 ) {
                    l_queue->next = Rq_head;
                    Rq_head = l_queue;
                    temp_Rq_head = l_queue;
            } else { // keep Rq_head
                    l_queue->next = temp_Rq_head->next;
                    temp_Rq_head->next = l_queue;
                    temp_Rq_head = l_queue;
            }
        }

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

        if ( head_or_tail == TAIL ) { // i am the dst
            // this pkt may have already queued
            if ( Rq_head != NULL && flag == RIGHT ) {
                temp_queue = Rq_head;
                while ( temp_queue != NULL ) {
                    if ( temp_queue->index == ch->uid() ) {
                        if ( temp_queue->flag == WRONG ) temp_queue->flag = RIGHT;
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
            } else {
                Rq_tail->next = l_queue;
                Rq_tail = l_queue;
            }
        }


        if ( flag == RIGHT ) cur_correct_in_Rq ++;
        cur_Rq_size ++;

        return;
}




void
MacRipple::update_data_daemon_to_send(  int forwarding )
{
        hdr_cmn                     * ch_daemon       = HDR_CMN(data_daemon);
        hdr_cmn                     * ch_pkt_in_Sq    = NULL;
        struct ripple_mac_data_hdr  * mh_daemon       = RIPPLE_MAC_DATA_HDR(data_daemon);
        struct ripple_mac_data_hdr  * mh_pkt_in_Sq    = NULL;
        struct mac_queue            * l_queue         = NULL;
        int                           daemon_size     = 0;
        int                           daemon_num_pkts = 0;
        int                           daemon_duration = 0;
        int                           daemon_dst;
        int                           temp;

        if ( ch_daemon->ptype() != PT_MAC_RIPPLE_DATA ) {
        	printf("update_data_daemon_to_send, MAC ACK should not be here, quit! in=%d. \n", index_);
        	exit(0);
        }

        if ( forwarding == WRONG ) {
        	// I am src, Sq != NULL
        	if ( Sq_head == NULL ) {
        		printf("update_data_daemon_to_send, src %d has no pkt to send, quit!\n", index_);
        		exit(0);
        	}

        	alloc_data_part_memory(data_daemon); // free memory in the data part only

            // put pkts in the frame
            temp      = daemon_num_pkts;
            l_queue   = Sq_head;
            for ( int i=0; i<ltjmin(MAX_FRAGMENTS,cur_Sq_size); i++) {
                if (  l_queue->flag != ACKED ) {
                       if ( daemon_num_pkts == temp ) {
                           daemon_dst    = l_queue->e2edst;
                           mh_pkt_in_Sq  = RIPPLE_MAC_DATA_HDR(l_queue->NSpkt);
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

            ch_daemon->size()      = daemon_size + mac_data_size();

            if ( mh_pkt_in_Sq != NULL )
              for ( int i=0; i<MAX_FORWARDERS; i++) {
                  ch_daemon->f_list[i] = ch_pkt_in_Sq->f_list[i];
              }
        }

        if ( forwarding == RIGHT ) {
            // I am a forwarder, search Sq for more pkts for the same dst, if the frame is not big enough
            daemon_num_pkts = ch_daemon->num_pkts();
            daemon_size     = ch_daemon->size();
            daemon_dst      = pkt_e2edst(data_daemon);

            if ( daemon_num_pkts < MAX_FRAGMENTS ) {
                l_queue = Sq_head;
                temp = daemon_num_pkts;
                while ( l_queue != NULL && daemon_num_pkts < MAX_FRAGMENTS ) {
                    if ( l_queue->flag != ACKED ){
                       // only forward pkts to a same direction
                       mh_pkt_in_Sq = RIPPLE_MAC_DATA_HDR(l_queue->NSpkt);
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
        }

        ch_daemon->direction() = hdr_cmn::DOWN;
        ch_daemon->txtime()    = txtime(ch_daemon->size(), dataRate_);
        ch_daemon->error()     = 0;

        mh_daemon->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
        mh_daemon->dh_fc.fc_type             = MAC_Type_Data;
        mh_daemon->dh_fc.fc_subtype          = MAC_Subtype_Data;
        mh_daemon->dh_fc.fc_to_ds            = 0;
        mh_daemon->dh_fc.fc_from_ds          = 0;
        mh_daemon->dh_fc.fc_more_frag        = 0;
        mh_daemon->dh_fc.fc_retry            = 0;
        mh_daemon->dh_fc.fc_pwr_mgt          = 0;
        mh_daemon->dh_fc.fc_more_data        = 0;
        mh_daemon->dh_fc.fc_wep              = 0;
        mh_daemon->dh_fc.fc_order            = 0;

        temp                   = hops_to_dst(data_daemon);
        daemon_duration        = usec((double)temp*ch_daemon->txtime());
        mh_daemon->dh_duration = daemon_duration;
        ch_daemon->num_pkts()  = daemon_num_pkts;

        STORE4BYTE(&daemon_dst, (mh_daemon->dh_ra));
        STORE4BYTE(&index_,     (mh_daemon->dh_ta));
        return ;
}


void
MacRipple::update_ack_daemon_to_send( int forwarding, Packet *pkt_rcved )
{
        hdr_cmn                     *  ch_ack_daemon    = HDR_CMN(ack_daemon);
        struct ripple_mac_ack_hdr   *  mh_ack_daemon    = RIPPLE_MAC_ACK_HDR(ack_daemon);
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

            // Construct ACK bitmap and pIDs
            construct_ACK_bitmap_pIDs( );
            for ( int i=0; i<MAX_FORWARDERS; i++) {
            	mh_ack_daemon->ripple_ack_hdr.ack_bitmap[i] = rq_ack_bitmap[i];
            }
            for ( int i=0; i<MAX_FRAGMENTS; i++) {
            	mh_ack_daemon->ripple_ack_hdr.pIDs[i] = rq_ack_pIDs[i];
            }

            ch_ack_daemon->direction() = hdr_cmn::DOWN;
            ch_ack_daemon->txtime()    = txtime(ch_ack_daemon->size(), dataRate_);
            ch_ack_daemon->error()     = 0;

            temp                       = hops_to_dst_ack(ack_daemon);
            tx_duration                = usec((double)temp*ch_ack_daemon->txtime());
            mh_ack_daemon->dh_duration = tx_duration;

            STORE4BYTE(&ch_ack_daemon->next_hop_, (mh_ack_daemon->dh_ra));
            STORE4BYTE(&index_,                   (mh_ack_daemon->dh_ta));
        } else {
        	// I am a forwarder, update ack_daemon
            ch_ack_daemon->direction() = hdr_cmn::DOWN;
            ch_ack_daemon->txtime()    = txtime(ch_ack_daemon->size(), dataRate_);
            ch_ack_daemon->error()     = 0;

            temp                       = hops_to_dst_ack(ack_daemon);
            tx_duration                = usec((double)temp*ch_ack_daemon->txtime());
            mh_ack_daemon->dh_duration = tx_duration;

            STORE4BYTE(&ch_ack_daemon->next_hop_, (mh_ack_daemon->dh_ra));
            STORE4BYTE(&index_,                   (mh_ack_daemon->dh_ta));
        }
}


void
MacRipple::alloc_data_part_memory( Packet *p )
{
    if ( p->accessdata() != 0 )
    {
    	p->setdata(NULL);
    }
    p->allocdata(sizeof(struct data_part)*MAX_FRAGMENTS);
}

void
MacRipple::remove_data_part_content(Packet *p)
{
    if ( p->accessdata() != 0 )
    {
    	p->setdata(NULL);
    }
}

void
MacRipple::save_into_data_part(struct mac_queue *l_queue, int whichone)
{
    PacketData        *l_data;
    struct data_part  *l_data_p;

    l_data   = (PacketData *)data_daemon->userdata();
    l_data_p = (struct data_part *)l_data->access_data(whichone*sizeof(struct data_part));

    l_data_p->NSpkt = l_queue->NSpkt;
}



void
MacRipple::clear_ACK_bitmap_pIDs( )
{
    for ( int i=0; i<MAX_FORWARDERS; i++) {
        rq_ack_bitmap[i] = (u_int16_t)0;
        rq_ack_pIDs[i]   = (int16_t)-2;
    }
}

void
MacRipple::construct_ACK_bitmap_pIDs( )
{
    struct mac_queue *  l_queue;
    u_int16_t           temp;
    u_int16_t           temp1 = (u_int16_t) 1;
    int                 howmany[MAX_FORWARDERS];
    int                 total_sum = 0;

    // clear the bitmap first
    clear_ACK_bitmap_pIDs();

    for ( int i=0; i<MAX_FORWARDERS; i++) {
        howmany[i]       = 0;
    }

    l_queue = Rq_head;
    while ( l_queue != NULL && total_sum < MAX_FRAGMENTS ) {
        if ( l_queue->flag == RIGHT ) {
            temp = temp1 << howmany[l_queue->e2esrc];
            rq_ack_bitmap[l_queue->e2esrc] = rq_ack_bitmap[l_queue->e2esrc] | temp;
            rq_ack_pIDs[total_sum]         = HDR_CMN(l_queue->NSpkt)->uid();
            total_sum ++;
        }
        howmany[l_queue->e2esrc] ++;
        l_queue = l_queue->next;
    }
}



int
MacRipple::hear_a_higher_pri_tx(Packet *p)
{
        struct hdr_cmn              * ch              = HDR_CMN(p);
        struct ripple_mac_data_hdr  * mdh             = RIPPLE_MAC_DATA_HDR(p);
        struct ripple_mac_ack_hdr   * mah             = RIPPLE_MAC_ACK_HDR(p);
        int                           my_dist_to_dst  = hops_to_dst(p);
        int                           p_dist_to_dst   = -1;
        int                           src;

        if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) src = ETHER_ADDR(mdh->dh_ta);
        if ( ch->ptype() == PT_MAC_RIPPLE_ACK )  src = ETHER_ADDR(mah->dh_ta);


        for ( int i=MAX_FORWARDERS-1; i>=0; i--) {
            if ( ch->f_list[i] >= 0 ) {
                p_dist_to_dst ++;
            }
            if ( ch->f_list[i] == src ) {
                break;
            }
        }

        if ( my_dist_to_dst > p_dist_to_dst )
                return RIGHT;

        return WRONG;
}




int
MacRipple::am_i_forwarder(Packet *p)
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
MacRipple::am_i_in_f_list(Packet *p)
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
MacRipple::my_priority(Packet *p)
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
        printf("MacRipple::my_priority, should not reach here! in=%d\n", index_);
        exit(0);
}

Packet *
MacRipple::reconstruct_data_daemon(  )
{
	struct hdr_cmn                 * ch        = HDR_CMN(data_daemon);
	PacketData                     * l_data;
	struct data_part               * l_data_p;
	Packet                         * l_NSpkt   = NULL;

	// free NS pkts in data_daemon
	l_data = (PacketData *)data_daemon->userdata();
	for ( int i=0; i<ch->num_pkts(); i++) {
			l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
			l_NSpkt  = l_data_p->NSpkt;
			Packet::free(l_NSpkt);
	}


	// free memory in data part of data_daemon used by outgoing packets
	 alloc_data_part_memory(data_daemon);
	 // renew data_daemon
	 Packet::free(data_daemon);
	 return ( construct_data_daemon() );
}


Packet *
MacRipple::construct_data_daemon( )
{
            Packet                     * p     = Packet::alloc();
            hdr_cmn                    * ch    = HDR_CMN(p);
            struct ripple_mac_data_hdr * mdh   = RIPPLE_MAC_DATA_HDR(p);

            alloc_data_part_memory (p);

            ch->ptype() = PT_MAC_RIPPLE_DATA;
            ch->iface() = -2;
            ch->error() = 0;

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
MacRipple::construct_ack_daemon( )
{
            Packet                    * p      = Packet::alloc();
            hdr_cmn                   * ch     = HDR_CMN(p);
            struct ripple_mac_ack_hdr * mah    = RIPPLE_MAC_ACK_HDR(p);

            ch->ptype() = PT_MAC_RIPPLE_ACK;
            ch->iface() = -2;
            ch->error() = 0;
//            ch->uid()   = -1;
            ch->uid()   = temp_seq_mac_ack --;
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
MacRipple::ltjmin(int a, int b)
{
    if ( a < b )
        return a;
    return b;
}


void
MacRipple::recv_correct_pkts(NsObject *uptarget)
{
    struct mac_queue *     l_queue;

    l_queue = Rq_head;
    while ( l_queue != NULL ){
        if ( l_queue->flag == RIGHT ) {
            HDR_CMN(l_queue->NSpkt)->direction() = hdr_cmn::UP;
            uptarget->recv(l_queue->NSpkt, (Handler*) 0);
#ifdef TIANJI_NEW_DEBUG
printf("Finish recv, uid=%d, %d\n", HDR_CMN(l_queue->NSpkt)->uid(), index_);
#endif
            remove_Rq_head( );
            cur_Rq_size --;
            cur_correct_in_Rq --;
        } else
            return; // only pass consecutively correct pkts to upper layer
        l_queue = l_queue->next;
    }
}

void
MacRipple::remove_Rq_head( )
{
        struct mac_queue * l_queue;

        if ( cur_Rq_size == 1 )
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
MacRipple::clear_ACKED_in_Rq( )
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
MacRipple::remove_Sq_head( )
{
        struct mac_queue * l_queue;

        if ( cur_Sq_size == 1 )
        {
                free(Sq_head);
                Sq_head = NULL;
                Sq_tail = NULL;
        }else
        {
                l_queue = Sq_head->next;
                free(Sq_head);
                Sq_head = l_queue;
        }
        return;
}


void
MacRipple::update_flags_in_Sq(Packet *p)
{
    if ( Sq_head == NULL ) return;

    struct mac_queue          * l_queue = Sq_head;
    struct ripple_mac_ack_hdr * mh      = RIPPLE_MAC_ACK_HDR(p);
    u_int16_t                   bitmap  = mh->ripple_ack_hdr.ack_bitmap[index_];
    u_int16_t                   temp1   = (u_int16_t)1;
    u_int16_t                   temp;

    for ( u_int16_t i=0; i<cur_Sq_size; i++ ) {
        temp = temp1 << i;
        temp = bitmap & temp;
        if ( temp > 0 && l_queue->flag == UNACKED ) {
            l_queue->flag = ACKED;
        }
        l_queue = l_queue->next;
    }

}

// remove all successfully transmitted pkts in the Sq
void
MacRipple::clear_ACKED_in_Sq( )
{
        struct mac_queue       * l_queue = NULL;

        if ( cur_Sq_size == 0 )
        {
          return;
        }else if ( cur_Sq_size == 1 ) {
			l_queue = Sq_head;
			if ( l_queue->flag == ACKED ) {
					free(Sq_head);
					Sq_head = NULL;
					Sq_tail = NULL;
					cur_Sq_size = 0;
			}
        }else {
            while ( Sq_head != NULL && Sq_head->flag == ACKED ) {
                remove_Sq_head();
                cur_Sq_size --;
            }
        }
        return;
}



// remove all successfully transmitted pkts in the Rq
void
MacRipple::clear_repeated_MAC_ACKs_in_Rq( )
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
MacRipple::print_mac_queue(char *fname, struct mac_queue *q)
{
    struct mac_queue           * l_queue;
    struct hdr_cmn             * l_ch;
    struct ripple_mac_data_hdr * mh;


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
            mh   = RIPPLE_MAC_DATA_HDR(l_queue->NSpkt);
            fprintf(stdout, "\t uid/flag/s/d/size: %d %d %d %d %d\n",
            		l_ch->uid(), l_queue->flag, ETHER_ADDR(mh->dh_ta), ETHER_ADDR(mh->dh_ra), l_ch->size());
            l_queue = l_queue->next;
        }
        return;
}

int
MacRipple::all_acked(struct mac_queue *q)
{
    struct mac_queue *l_queue;
    int              temp = 0;

    l_queue = q;

    while ( l_queue != NULL && temp < MAX_FRAGMENTS ) {
        if ( l_queue->flag != ACKED )
            return WRONG;
        l_queue = l_queue->next;
        temp ++;
    }

    return RIGHT;
}


int
MacRipple::all_correct(struct mac_queue *q)
{
    struct mac_queue *l_queue;

    l_queue = q;

    while ( l_queue != NULL ) {
        if ( l_queue->flag != RIGHT )
            return WRONG;
        l_queue = l_queue->next;
    }

    return RIGHT;
}

int
MacRipple::hops_to_dst(Packet *p)
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
    printf("MacRipple::hops_to_dst, should not reach here! in=%d\n", index_);
    exit(0);
}

int
MacRipple::hops_to_dst_ack(Packet *p)
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
    printf("MacRipple::hops_to_dst_ack, should not reach here! in=%d\n", index_);
    exit(0);
}


void
MacRipple::free_ripple_pkt(Packet *p)
{
	struct hdr_cmn                 * ch        = HDR_CMN(p);
	PacketData                     * l_data;
	struct data_part               * l_data_p;
	Packet                         * l_NSpkt   = NULL;

	 if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
		// free NS pkts in data_daemon
		l_data = (PacketData *)p->userdata();
		for ( int i=0; i<ch->num_pkts(); i++) {
				l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
				l_NSpkt  = l_data_p->NSpkt;
				Packet::free(l_NSpkt);
		}

		remove_data_part_content(p);
		Packet::free(p);
		p = 0;
	 }

     if ( ch->ptype() == PT_MAC_RIPPLE_ACK ) {
    	Packet::free(p);
		p = 0;
	 }
}