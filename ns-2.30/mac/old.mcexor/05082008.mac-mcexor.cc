/* -*-  Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
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
 *      This product includes software developed by the Computer Systems
 *      Engineering Group at Lawrence Berkeley Laboratory.
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
 * $Header: /tianji.cvsroot/newripple/ns-2.30/mac/mcexor/05082008.mac-mcexor.cc,v 1.1.1.1 2008/12/09 11:15:35 ripple Exp $
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

#define TIANJI_DEBUG_8

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
        tx_active_ = 1;
        struct hdr_cmn *ch = HDR_CMN(p);
        struct hdr_mac_mcexor *dh = HDR_MAC_MCEXOR(p);
        int dst, src;
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt = NULL;
     
        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        // get the pointer of pkts in this frame
        l_data = (PacketData *)p->userdata();
        l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
        l_NSpkt = l_data_p->NSpkt;


#ifdef TIANJI_DEBUG_8
printf("\n\n\nxmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf, to=%lf\n", index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), timeout);
trace_pkt("xmit",p);
print_mac_queue("xmit", Sq_head);
#endif

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
#ifdef TIANJI_DEBUG_8
printf("transmit, is this happening? %d\n", index_);
#endif
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
        downtarget_->recv(p->copy(), this);     
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

PHY_MIB::PHY_MIB(MacMCExOR *parent)
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

MAC_MIB::MAC_MIB(MacMCExOR *parent)
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
        tcl.evalf("Mac/MCExOR set basicRate_");
        if (strcmp(tcl.result(), "0") != 0) 
                bind_bw("basicRate_", &basicRate_);
        else
                basicRate_ = bandwidth_;

        tcl.evalf("Mac/MCExOR set dataRate_");
        if (strcmp(tcl.result(), "0") != 0) 
                bind_bw("dataRate_", &dataRate_);
        else
                dataRate_ = bandwidth_;

        bind_bool("bugFix_timer_", &bugFix_timer_);

        EOTtarget_ = 0;
        bss_id_ = IBSS_ID;
        //printf("bssid in constructor %d\n",bss_id_);

        daemon = construct_daemon();
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
        
        //char *src_nodeaddr =
        //       Address::instance().print_nodeaddr(iph->saddr());
        //char *dst_nodeaddr =
        //      Address::instance().print_nodeaddr(iph->daddr());
        
        struct hdr_mac_mcexor* dh = HDR_MAC_MCEXOR(p);
        
        //struct hdr_cmn *ch = HDR_CMN(p);
        
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
        struct hdr_cmn            *ch       = HDR_CMN(p);
        struct hdr_mac_mcexor     *dh       = HDR_MAC_MCEXOR(p);
        u_int16_t                 *t        = (u_int16_t*) &dh->dh_fc;
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt  = NULL;
        struct hdr_cmn            *l_ch;

        
        fprintf(stdout,
                "%s --- (INDEX: %d, time: %2.9f)\n",
                fname, index_, Scheduler::instance().clock());
        fprintf(stdout, "\t dur/src/dst/type/size/n:%d %d %d %s %d %d\n",
                 dh->dh_duration,
                 ETHER_ADDR(dh->dh_ta), ETHER_ADDR(dh->dh_ra),
                 packet_info.name(ch->ptype()), ch->size(), ch->num_pkts());
        fprintf(stdout, "\t f.list: %d %2d %d %d %d\n",
                dh->mcexor_hdr.f_list[0], 
                dh->mcexor_hdr.f_list[1], 
                dh->mcexor_hdr.f_list[2], 
                dh->mcexor_hdr.f_list[3], 
                dh->mcexor_hdr.f_list[4]
                );
        fprintf(stdout, "\t ack_bitmap: %d %d %d %d %d\n",
                dh->mcexor_hdr.ack_bitmap[0], 
                dh->mcexor_hdr.ack_bitmap[1], 
                dh->mcexor_hdr.ack_bitmap[2], 
                dh->mcexor_hdr.ack_bitmap[3], 
                dh->mcexor_hdr.ack_bitmap[4]
                );

        l_data = (PacketData *)p->userdata();
        for ( int i=0; i<ch->num_pkts(); i++) {
                l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
                l_NSpkt  = l_data_p->NSpkt;
                l_ch     = HDR_CMN(l_NSpkt);
                fprintf(stdout, "\t uid/es/ed/size:%d %d %d %d \n",
                        l_ch->uid(), 
                        l_ch->prev_hop_, 
                        l_ch->next_hop_, 
                        l_ch->size()
                        );
        }

}

void
MacMCExOR::dump(char *fname)
{
        fprintf(stdout,
                "%s --- (INDEX: %d, time: %2.9f)\n",
                fname, index_, Scheduler::instance().clock());

        fprintf(stdout,
                "\ttx_state_: %x, rx_state_: %x, nav: %2.9f, idle: %d\n",
                tx_state_, rx_state_, nav_, is_idle());

        fprintf(stdout,
                "\tTx_: %lx, Rx_: %lx, daemon: %lx,  CTRL_: %lx, callback: %lx\n",
                (long) pktTx_, (long) pktRx_, (long) daemon, 
                (long) pktCTRL_, (long) callback_);

        fprintf(stdout,
                "\tDefer: %d, Backoff: %d (%d), Recv: %d, Send: %d If: %d Nav: %d\n",
                mhDefer_.busy(), mhBackoff_.busy(), mhBackoff_.paused(),
                mhRecv_.busy(), mhSend_.busy(), mhIF_.busy(), mhNav_.busy());
        fprintf(stdout,
                "\tBackoff Expire: %f\n",
                mhBackoff_.expire());
}


/* ======================================================================
   Packet Headers Routines
   ====================================================================== */
inline int
MacMCExOR::hdr_dst(char* hdr, int dst )
{
        struct hdr_mac_mcexor *dh = (struct hdr_mac_mcexor*) hdr;
        
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
        struct hdr_mac_mcexor *dh = (struct hdr_mac_mcexor*) hdr;
        if(src > -2)
               STORE4BYTE(&src, (dh->dh_ta));
        return ETHER_ADDR(dh->dh_ta);
}

inline int 
MacMCExOR::hdr_type(char* hdr, u_int16_t type)
{
        struct hdr_mac_mcexor *dh = (struct hdr_mac_mcexor*) hdr;
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
        hdr_mac_mcexor* mh = HDR_MAC_MCEXOR(p);
        hdr_cmn *ch = HDR_CMN(p);

        /* if the rcvd pkt contains errors, a real MAC layer couldn't
           necessarily read any data from it, so we just toss it now */
        if(ch->error() != 0) {
                Packet::free(p);
                return;
        }

        switch(mh->dh_fc.fc_type) {
        case MAC_Type_Management:
                drop(p, why);
                return;
        case MAC_Type_Control:
                switch(mh->dh_fc.fc_subtype) {
                case MAC_Subtype_RTS:
                         if((u_int32_t)ETHER_ADDR(mh->dh_ta) ==  (u_int32_t)index_) {
                                drop(p, why);
                                return;
                        }
                        /* fall through - if necessary */
                case MAC_Subtype_CTS:
                case MAC_Subtype_ACK:
                        if((u_int32_t)ETHER_ADDR(mh->dh_ra) == (u_int32_t)index_) {
                                drop(p, why);
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
                                drop(p,why);
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
//              trace_pkt(p);
                exit(1);
        }
        Packet::free(p);
}

void
MacMCExOR::capture(Packet *p)
{
        /*
         * Update the NAV so that this does not screw
         * up carrier sense.
         */     
        set_nav(usec(phymib_.getEIFS() + txtime(p)));
        Packet::free(p);
}

void
MacMCExOR::collision(Packet *p)
{
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
                        discard(pktRx_, DROP_MAC_COLLISION);
                        pktRx_ = p;
                        mhRecv_.start(txtime(pktRx_));
                }
                else {
                        discard(p, DROP_MAC_COLLISION);
                }
                break;
        default:
                assert(0);
        }
}

void
MacMCExOR::tx_resume(int priority)
{
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::tx_resume, pri=%d, %d\n", priority, index_);
#endif
        double rTime;
        assert(mhSend_.busy() == 0);
        assert(mhDefer_.busy() == 0);

        if(pktCTRL_) {
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::tx_resume, pktCTRL, %d\n", index_);
#endif
                mhDefer_.start((double)priority*phymib_.getSIFS());
        } else if(pktRTS_) {
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::tx_resume, pktRTS_, %d\n", index_);
#endif
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
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::tx_resume, pktTx_, %d\n", index_);
#endif
                if (mhBackoff_.busy() == 0) {
                        hdr_cmn *ch = HDR_CMN(pktTx_);
                        struct hdr_mac_mcexor *mh = HDR_MAC_MCEXOR(pktTx_);
                        
                        if ((u_int32_t) ch->size() < macmib_.getRTSThreshold()
                            || ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
                                if (bugFix_timer_) {
                                        mhBackoff_.start(cw_, is_idle(), 
                                                         phymib_.getDIFS());
                                }
                                else {
                                        rTime = (Random::random() % cw_)
                                                * phymib_.getSlotTime();
                                        mhDefer_.start(phymib_.getDIFS() + 
                                                       rTime);
                                }
                        } else {
                                mhDefer_.start(phymib_.getSIFS());
                        }
                }
        } else if(callback_) {
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::tx_resume, callback_, %d\n", index_);
#endif
                Handler *h = callback_;
                callback_ = 0;
                h->handle((Event*) 0);
        }
        setTxState(MAC_IDLE);
#ifdef TIANJI_DEBUG_8
dump("end of tx_resume");
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
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::backoffHandler, %d\n", index_);
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
MacMCExOR::deferHandler()
{
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::deferHandler, now=%lf, %d\n", Scheduler::instance().clock(),index_);
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
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::txHandler, %d\n", index_);
#endif
        if (EOTtarget_) {
                assert(eotPacket_);
                EOTtarget_->recv(eotPacket_, (Handler *) 0);
                eotPacket_ = NULL;
        }
        tx_active_ = 0;
}


/* ======================================================================
   The "real" Timer Handler Routines
   ====================================================================== */
void
MacMCExOR::send_timer()
{
#ifdef TIANJI_DEBUG_8
printf("send_timer, %d\n", index_);
dump("send_timer");
#endif
        switch(tx_state_) {
        /*
         * Sent a RTS, but did not receive a CTS.
         */
        case MAC_RTS:
                RetransmitRTS();
                break;
        /*
         * Sent a CTS, but did not receive a DATA packet.
         */
        case MAC_CTS:
                assert(pktCTRL_);
                Packet::free(pktCTRL_); 
                pktCTRL_ = 0;
                break;
        /*
         * Sent DATA, but did not receive an ACK packet.
         */
        case MAC_SEND:
                if ( rx_state_ != MAC_IDLE  ) { 
                    if ( mhIF_.busy() == 1) { // do not retx if hear sth before finishing sending
    //                     mhBackoff_.start(cw_, is_idle(), phymib_.getDIFS());                    pktTx_ = 0;
                        // backoff can be shorter than needed, use long defer instead
                        mhDefer_.start(sec(HDR_MAC_MCEXOR(pktRx_)->dh_duration)+hops_to_dst(pktTx_)*phymib_.getSIFS()+phymib_.getDIFS());
                    } else {
                        pktTx_ = 0;
                        setTxState(MAC_IDLE);
                    }
#ifdef TIANJI_DEBUG_8
dump("retx");
#endif
                    break; 
                }
                RetransmitDATA();
                break;
        /*
         * Sent an ACK, and now ready to resume transmission.
         */
        case MAC_ACK:
                assert(pktCTRL_);
//              Packet::free(pktCTRL_); 
                pktCTRL_ = 0;
//                 pktTx_ = 0;
                break;
        case MAC_IDLE:
                break;
        default:
                assert(0);
        }
        tx_resume(-1);
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
MacMCExOR::check_pktCTRL()
{
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::check_pktCTRL, %d\n", index_);
#endif
        struct hdr_mac_mcexor *mh;
        double timeout;
        int temp;

        if(pktCTRL_ == 0)
                return -1;
        if(tx_state_ == MAC_CTS || tx_state_ == MAC_ACK)
                return -1;

        mh = HDR_MAC_MCEXOR(pktCTRL_);
                                                          
        switch(mh->dh_fc.fc_subtype) {
        /*
         *  If the medium is not IDLE, don't send the CTS.
         */
        case MAC_Subtype_CTS:
                if(!is_idle()) {
                        discard(pktCTRL_, DROP_MAC_BUSY); pktCTRL_ = 0;
                        return 0;
                }
                setTxState(MAC_CTS);
        
                 timeout = txtime(phymib_.getCTSlen(), basicRate_)
                        + DSSS_MaxPropagationDelay                      // XXX
                        + sec(mh->dh_duration)
                        + DSSS_MaxPropagationDelay                      // XXX
                       - phymib_.getSIFS()
                       - txtime(phymib_.getACKlen(), basicRate_);
                break;
                /*
                 * IEEE 802.11 specs, section 9.2.8
                 * Acknowledments are sent after an SIFS, without regard to
                 * the busy/idle state of the medium.
                 */
        case MAC_Subtype_ACK:           
                setTxState(MAC_ACK);
                timeout = txtime(phymib_.getACKlen(), basicRate_);
                break;
        case MAC_Subtype_Data:    // mcexor is sending reverse-direction frames       
                setTxState(MAC_ACK);
                temp = hops_to_dst(pktCTRL_);
                timeout = temp * (DSSS_MaxPropagationDelay + phymib_.getSIFS())
                          + sec(mh->dh_duration);                       
                break;
        default:
                fprintf(stderr, "check_pktCTRL:Invalid MAC Control subtype\n");
                exit(1);
        }
        transmit(pktCTRL_, timeout);
        return 0;
}

int
MacMCExOR::check_pktRTS()
{
        struct hdr_mac_mcexor *mh;
        double timeout;

        assert(mhBackoff_.busy() == 0);

        if(pktRTS_ == 0)
                return -1;
        mh = HDR_MAC_MCEXOR(pktRTS_);

        switch(mh->dh_fc.fc_subtype) {
        case MAC_Subtype_RTS:
                if(! is_idle()) {
                        inc_cw();
                        mhBackoff_.start(cw_, is_idle());
                        return 0;
                }
                setTxState(MAC_RTS);
                timeout = txtime(phymib_.getRTSlen(), basicRate_)
                        + DSSS_MaxPropagationDelay                      // XXX
                        + phymib_.getSIFS()
                        + txtime(phymib_.getCTSlen(), basicRate_)
                        + DSSS_MaxPropagationDelay;
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
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::check_pktTx, %d\n", index_);
#endif
        struct hdr_mac_mcexor *mh;
        double timeout;
        int temp;
        
        assert(mhBackoff_.busy() == 0);

// there may be more pkts available during backoff
        get_a_frm_to_send(WRONG);
        if ( HDR_CMN(daemon)->num_pkts() > 0 )
            pktTx_ = daemon;    
        else 
            pktTx_ = 0;

        if(pktTx_ == 0)
                return -1;

        mh = HDR_MAC_MCEXOR(pktTx_);

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
               } else
                        timeout = txtime(pktTx_);
                break;
        default:
                fprintf(stderr, "check_pktTx:Invalid MAC Control subtype\n");
                exit(1);
        }
        transmit(pktTx_, timeout);
        return 0;
}
/*
 * Low-level transmit functions that actually place the packet onto
 * the channel.
 */
void
MacMCExOR::sendRTS(int dst)
{
        Packet *p = Packet::alloc();
        hdr_cmn* ch = HDR_CMN(p);
        struct rts_frame *rf = (struct rts_frame*)p->access(hdr_mac::offset_);
        
        assert(pktTx_);
        assert(pktRTS_ == 0);

        /*
         *  If the size of the packet is larger than the
         *  RTSThreshold, then perform the RTS/CTS exchange.
         */
        if( (u_int32_t) HDR_CMN(pktTx_)->size() < macmib_.getRTSThreshold() ||
             dst == MAC_BROADCAST) {
                Packet::free(p);
                return;
        }

        ch->uid() = 0;
        ch->ptype() = PT_MAC;
        ch->size() = phymib_.getRTSlen();
        ch->iface() = -2;
        ch->error() = 0;

        bzero(rf, MAC_HDR_LEN);

        rf->rf_fc.fc_protocol_version = MAC_ProtocolVersion;
        rf->rf_fc.fc_type       = MAC_Type_Control;
        rf->rf_fc.fc_subtype    = MAC_Subtype_RTS;
        rf->rf_fc.fc_to_ds      = 0;
        rf->rf_fc.fc_from_ds    = 0;
        rf->rf_fc.fc_more_frag  = 0;
        rf->rf_fc.fc_retry      = 0;
        rf->rf_fc.fc_pwr_mgt    = 0;
        rf->rf_fc.fc_more_data  = 0;
        rf->rf_fc.fc_wep        = 0;
        rf->rf_fc.fc_order      = 0;

        //rf->rf_duration = RTS_DURATION(pktTx_);
        STORE4BYTE(&dst, (rf->rf_ra));
        
        /* store rts tx time */
        ch->txtime() = txtime(ch->size(), basicRate_);
        
        STORE4BYTE(&index_, (rf->rf_ta));

        /* calculate rts duration field */      
        rf->rf_duration = usec(phymib_.getSIFS()
                               + txtime(phymib_.getCTSlen(), basicRate_)
                               + phymib_.getSIFS()
                               + txtime(pktTx_)
                               + phymib_.getSIFS()
                               + txtime(phymib_.getACKlen(), basicRate_));
        pktRTS_ = p;
}

void
MacMCExOR::sendCTS(int dst, double rts_duration)
{
        Packet *p = Packet::alloc();
        hdr_cmn* ch = HDR_CMN(p);
        struct cts_frame *cf = (struct cts_frame*)p->access(hdr_mac::offset_);

        assert(pktCTRL_ == 0);

        ch->uid() = 0;
        ch->ptype() = PT_MAC;
        ch->size() = phymib_.getCTSlen();


        ch->iface() = -2;
        ch->error() = 0;
        //ch->direction() = hdr_cmn::DOWN;
        bzero(cf, MAC_HDR_LEN);

        cf->cf_fc.fc_protocol_version = MAC_ProtocolVersion;
        cf->cf_fc.fc_type       = MAC_Type_Control;
        cf->cf_fc.fc_subtype    = MAC_Subtype_CTS;
        cf->cf_fc.fc_to_ds      = 0;
        cf->cf_fc.fc_from_ds    = 0;
        cf->cf_fc.fc_more_frag  = 0;
        cf->cf_fc.fc_retry      = 0;
        cf->cf_fc.fc_pwr_mgt    = 0;
        cf->cf_fc.fc_more_data  = 0;
        cf->cf_fc.fc_wep        = 0;
        cf->cf_fc.fc_order      = 0;
        
        //cf->cf_duration = CTS_DURATION(rts_duration);
        STORE4BYTE(&dst, (cf->cf_ra));
        
        /* store cts tx time */
        ch->txtime() = txtime(ch->size(), basicRate_);
        
        /* calculate cts duration */
        cf->cf_duration = usec(sec(rts_duration)
                              - phymib_.getSIFS()
                              - txtime(phymib_.getCTSlen(), basicRate_));


        
        pktCTRL_ = p;
        
}

void
MacMCExOR::sendACK(int dst)
{
        Packet *p = Packet::alloc();
        hdr_cmn* ch = HDR_CMN(p);
        struct ack_frame *af = (struct ack_frame*)p->access(hdr_mac::offset_);

        assert(pktCTRL_ == 0);

        ch->uid() = 0;
        ch->ptype() = PT_MAC;
        // CHANGE WRT Mike's code
        ch->size() = phymib_.getACKlen();
        ch->iface() = -2;
        ch->error() = 0;
        
        bzero(af, MAC_HDR_LEN);

        af->af_fc.fc_protocol_version = MAC_ProtocolVersion;
        af->af_fc.fc_type       = MAC_Type_Control;
        af->af_fc.fc_subtype    = MAC_Subtype_ACK;
        af->af_fc.fc_to_ds      = 0;
        af->af_fc.fc_from_ds    = 0;
        af->af_fc.fc_more_frag  = 0;
        af->af_fc.fc_retry      = 0;
        af->af_fc.fc_pwr_mgt    = 0;
        af->af_fc.fc_more_data  = 0;
        af->af_fc.fc_wep        = 0;
        af->af_fc.fc_order      = 0;

        //af->af_duration = ACK_DURATION();
        STORE4BYTE(&dst, (af->af_ra));

        /* store ack tx time */
        ch->txtime() = txtime(ch->size(), basicRate_);
        
        /* calculate ack duration */
        af->af_duration = 0;    
        
        pktCTRL_ = p;
}

void
MacMCExOR::sendDATA(Packet *p)
{
        assert(pktTx_ == 0);

        if ( daemon!=NULL )
        {
                pktTx_ = daemon;
        }else
        {
                printf("MacMCExOR::sendDATA, no frame to send, quit...%d\n",index_);
                exit(0);
        }

}

/* ======================================================================
   Retransmission Routines
   ====================================================================== */
void
MacMCExOR::RetransmitRTS()
{
        assert(pktTx_);
        assert(pktRTS_);
        assert(mhBackoff_.busy() == 0);
        macmib_.RTSFailureCount++;


        ssrc_ += 1;                     // STA Short Retry Count
                
        if(ssrc_ >= macmib_.getShortRetryLimit()) {
                discard(pktRTS_, DROP_MAC_RETRY_COUNT_EXCEEDED); pktRTS_ = 0;
                /* tell the callback the send operation failed 
                   before discarding the packet */
                hdr_cmn *ch = HDR_CMN(pktTx_);
                if (ch->xmit_failure_) {
                        /*
                         *  Need to remove the MAC header so that 
                         *  re-cycled packets don't keep getting
                         *  bigger.
                         */
                        ch->size() -= phymib_.getHdrLen11();
                        ch->xmit_reason_ = XMIT_REASON_RTS;
                        ch->xmit_failure_(pktTx_->copy(),
                                          ch->xmit_failure_data_);
                }
                discard(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED); 
                pktTx_ = 0;
                ssrc_ = 0;
                rst_cw();
        } else {
                struct rts_frame *rf;
                rf = (struct rts_frame*)pktRTS_->access(hdr_mac::offset_);
                rf->rf_fc.fc_retry = 1;

                inc_cw();
                mhBackoff_.start(cw_, is_idle());
        }
}

void
MacMCExOR::RetransmitDATA()
{
        struct hdr_cmn *ch;
        struct hdr_mac_mcexor *mh;
        u_int32_t *rcount, thresh;
        assert(mhBackoff_.busy() == 0);

        assert(pktTx_);
        assert(pktRTS_ == 0);

        ch = HDR_CMN(pktTx_);
        mh = HDR_MAC_MCEXOR(pktTx_);
//         int dst = ETHER_ADDR(mh->dh_ra);
        int src = ETHER_ADDR(mh->dh_ta);

        /*
         *  Broadcast packets don't get ACKed and therefore
         *  are never retransmitted.
         */
        if(ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
                Packet::free(pktTx_); 
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

#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::RetransmitDATA, rcount=%d, in=%d\n", *rcount, index_);
dump("RetransmitDATA");
trace_pkt("RetransmitDATA", pktTx_);
#endif

        (*rcount)++;

        if(*rcount >= thresh) {
                /* IEEE Spec section 9.2.3.5 says this should be greater than
                   or equal */
                macmib_.FailedCount++;
                /* tell the callback the send operation failed 
                   before discarding the packet */
                hdr_cmn *ch = HDR_CMN(pktTx_);
                if (ch->xmit_failure_) {
                        ch->size() -= phymib_.getHdrLen11();
                        ch->xmit_reason_ = XMIT_REASON_ACK;
                        ch->xmit_failure_(pktTx_->copy(),
                                          ch->xmit_failure_data_);
                }

//              discard(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED); 
//                 pktTx_ = 0;
                *rcount = 0;
// if collisions retrylimit times, do not do anything, just keep trying!!!
                rst_cw();
        }
        else {

                // form a new frame, as more pkts may be available
                if ( src == index_ )  {// not forwarding
                    if ( cur_Sq_size > 0  )  get_a_frm_to_send( WRONG ); 
                    else goto exit_retx; // do not retx MAC ACKs 
                } else if ( src < index_ ) { // forwarding
                    if ( cur_correct_in_Rq > 0 ) get_a_frm_to_send( RIGHT ); 
                    else goto exit_retx;  // should not happen?
                } else goto exit_retx; // do not retx MAC ACKs

                mh->dh_fc.fc_retry = 1;

                inc_cw();
                mhBackoff_.start(cw_, is_idle());
        }
        return;

exit_retx:
//         Packet::free(pktTx_);
// never free daemon 
        pktTx_ = 0;
        rst_cw();
        mhBackoff_.start(cw_, is_idle());
        return;

}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
MacMCExOR::send(Packet *p, Handler *h)
{
        double rTime;
        struct hdr_mac_mcexor* dh = HDR_MAC_MCEXOR(p);

//         rcount ++;

        EnergyModel *em = netif_->node()->energy_model();
        if (em && em->sleep()) {
                em->set_node_sleep(0);
                em->set_node_state(EnergyModel::INROUTE);
        }
        
        callback_ = h;
        sendDATA(p);
//      sendRTS(ETHER_ADDR(dh->dh_ra));

        /*
         * Assign the data packet a sequence number.
         */
        dh->dh_scontrol = sta_seqno_++;

        /*
         *  If the medium is IDLE, we must wait for a DIFS
         *  Space before transmitting.
         */
        if(mhBackoff_.busy() == 0) {
                if(is_idle()) {
                        if (mhDefer_.busy() == 0) {
                                /*
                                 * If we are already deferring, there is no
                                 * need to reset the Defer timer.
                                 */
                                if (bugFix_timer_) {
                                         mhBackoff_.start(cw_, is_idle(), 
                                                          phymib_.getDIFS());
                                }
                                else {
                                        rTime = (Random::random() % cw_)
                                                * (phymib_.getSlotTime());
                                        mhDefer_.start(phymib_.getDIFS() + 
                                                       rTime);
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
MacMCExOR::recv(Packet *p, Handler *h)
{
        struct hdr_cmn        *hdr   = HDR_CMN(p);
        struct hdr_mac_mcexor *mh    = HDR_MAC_MCEXOR(p);

        int dst    = ETHER_ADDR(mh->dh_ra);
        int src    = ETHER_ADDR(mh->dh_ta);
        int e2esrc = hdr->prev_hop_;
        int e2edst = hdr->next_hop_;
        assert(initialized());

        if(hdr->direction() == hdr_cmn::DOWN) {
#ifdef TIANJI_DEBUG_8
// printf("recv() down, uid=%d, size=%d, now=%lf, in=%d\n", hdr->uid(), hdr->size(), Scheduler::instance().clock(), index_);
#endif
                if ( cur_Sq_size < MAX_Sq_SIZE ) {
                     push_Sq(p); // drain the ifq and save in Sq
                } else {
                     discard(p, DROP_RTR_QFULL);
                }
    
                daemon_handler = h;
                callback_      = h;
// dump("recv down");

                get_a_frm_to_send(WRONG);
                send(daemon, daemon_handler);

                if ( cur_Sq_size < MAX_Sq_SIZE ) {
                    // fetch another pkt if any from the upper layer
                    Handler *h_temp = h;
                    h = 0;
                    h_temp->handle((Event*) 0);
                } 
                return;
        }


        if(tx_active_ && hdr->error() == 0) {
                hdr->error() = 1;
        }

#ifdef TIANJI_DEBUG_8
printf("recv() up before, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, bitmap=%d, in --- %d\n", hdr->prev_hop_, hdr->next_hop_, src, dst, hdr->size(), hdr->error(), keep_ACK_bitmap, index_);
dump("recv up before");
trace_pkt("recv up before", p);
#endif

// forwarders should cancel their tx if hear high priority tx
        if ( dst != index_ && pktCTRL_ != 0 ) {
            if(mhDefer_.busy()) 
                mhDefer_.stop();
            remove_data_part_content(daemon); // clear the data part if non-empty
            pktCTRL_ = 0;
        }

        if ( e2esrc < e2edst && src > index_ ) { // fd: hear a high priority data
            hdr->error() = 1; 
        }

        if ( e2esrc > e2edst && src < index_ ) { // rd: hear a high priority data
            hdr->error() = 1; 
        }

        // if rcv a pkt after sending a MAC ACK, re-send the bitmap next time
        if ( tx_state_ == MAC_ACK )
            keep_ACK_bitmap = RIGHT;
        else 
            keep_ACK_bitmap = WRONG;

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

#ifdef TIANJI_DEBUG_8
printf("recv() up, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, bitmap=%d, in --- %d\n", hdr->prev_hop_, hdr->next_hop_, src, dst, hdr->size(), hdr->error(), keep_ACK_bitmap, index_);
dump("recv up");
trace_pkt("recv up", p);
#endif
}

void
MacMCExOR::recv_timer()
{
        hdr_cmn *ch = HDR_CMN(pktRx_);
        hdr_mac_mcexor *mh = HDR_MAC_MCEXOR(pktRx_);
//      int dst = (int)ETHER_ADDR(mh->dh_ra);
        
        u_int8_t  type = mh->dh_fc.fc_type;
        u_int8_t  subtype = mh->dh_fc.fc_subtype;

        assert(pktRx_);
        assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);
        

        int dst, src;
        dst = ETHER_ADDR(mh->dh_ra);
        src = ETHER_ADDR(mh->dh_ta);
#ifdef TIANJI_DEBUG_8
printf("recv_timer, n=%d, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", ch->num_pkts(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), index_);
print_mac_queue("recv_timer", Rq_head);
dump("recv_timer");
#endif


        if(tx_active_) {
                Packet::free(pktRx_);
                goto done;
        }


        if(rx_state_ == MAC_COLL) {
                discard(pktRx_, DROP_MAC_COLLISION);            
                set_nav(usec(phymib_.getEIFS()));
                goto done;
        }


        if( ch->error() == 1 ) {
                Packet::free(pktRx_);
                set_nav(usec(phymib_.getEIFS()));
                goto done;
        }

// src/dst are also forwarders in this function
            if ( am_i_forwarder(pktRx_) == WRONG ) {
                    set_nav(mh->dh_duration);
            }
            
        

        /* tap out - */
        if (tap_ && type == MAC_Type_Data &&
            MAC_Subtype_Data == subtype ) 
                tap_->tap(pktRx_);
        /*
         * Adaptive Fidelity Algorithm Support - neighborhood infomation 
         * collection
         *
         * Hacking: Before filter the packet, log the neighbor node
         * I can hear the packet, the src is my neighbor
         */
        if (netif_->node()->energy_model() && 
            netif_->node()->energy_model()->adaptivefidelity()) {
                src = ETHER_ADDR(mh->dh_ta);
                netif_->node()->energy_model()->add_neighbor(src);
        }



#ifdef TIANJI_DEBUG_8
printf("recv_timer, uid=%d, size=%d, %d\n", ch->uid(), ch->size(), index_);
#endif
        switch(type) {

        case MAC_Type_Management:
                discard(pktRx_, DROP_MAC_PACKET_ERROR);
                goto done;
        case MAC_Type_Control:
                switch(subtype) {
                case MAC_Subtype_RTS:
                        recvRTS(pktRx_);
                        break;
                case MAC_Subtype_CTS:
                        recvCTS(pktRx_);
                        break;
                case MAC_Subtype_ACK:
                        recvACK(pktRx_);
                        break;
                default:
                        fprintf(stderr,"recvTimer1:Invalid MAC Control Subtype %x\n",
                                subtype);
                        exit(1);
                }
                break;
        case MAC_Type_Data:
                switch(subtype) {
                case MAC_Subtype_Data:
                        recvDATA(pktRx_);
                        break;
                default:
                        fprintf(stderr, "recv_timer2:Invalid MAC Data Subtype %x\n",
                                subtype);
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


void
MacMCExOR::recvRTS(Packet *p)
{
        struct rts_frame *rf = (struct rts_frame*)p->access(hdr_mac::offset_);

        if(tx_state_ != MAC_IDLE) {
                discard(p, DROP_MAC_BUSY);
                return;
        }

        /*
         *  If I'm responding to someone else, discard this RTS.
         */
        if(pktCTRL_) {
                discard(p, DROP_MAC_BUSY);
                return;
        }

        sendCTS(ETHER_ADDR(rf->rf_ta), rf->rf_duration);

        /*
         *  Stop deferring - will be reset in tx_resume().
         */
        if(mhDefer_.busy()) mhDefer_.stop();

        tx_resume(-1);

        mac_log(p);
}

/*
 * txtime()     - pluck the precomputed tx time from the packet header
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
 * txtime()     - calculate tx time for packet of size "psz" bytes 
 *                at rate "drt" bps
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
MacMCExOR::recvCTS(Packet *p)
{
        if(tx_state_ != MAC_RTS) {
                discard(p, DROP_MAC_INVALID_STATE);
                return;
        }

        assert(pktRTS_);
        Packet::free(pktRTS_); pktRTS_ = 0;

        assert(pktTx_); 
        mhSend_.stop();

        /*
         * The successful reception of this CTS packet implies
         * that our RTS was successful. 
         * According to the IEEE spec 9.2.5.3, you must 
         * reset the ssrc_, but not the congestion window.
         */
        ssrc_ = 0;
        tx_resume(-1);

        mac_log(p);
}

void
MacMCExOR::recvDATA(Packet *p)
{
        struct hdr_mac_mcexor *dh = HDR_MAC_MCEXOR(p);
        int dst, src, size;
        struct hdr_cmn *ch = HDR_CMN(p);

        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        size = ch->size();

struct hdr_cmn *l_ch = HDR_CMN(p);

// 1. generate more errors at MAC??? is shadowing model enough?
        double                    per;
        double                    var;
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt = NULL;

#ifdef TIANJI_DEBUG_8
printf("recvData, 11111, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", l_ch->prev_hop_, l_ch->next_hop_, src, dst, l_ch->size(), l_ch->error(), Scheduler::instance().clock(), index_);
#endif

        // src/dst update Sq according to the ACK bitmap
        if ( dst == index_ ) {
            update_flags_in_Sq(pktRx_);

            // get the pointer of pkts in this frame
            l_data = (PacketData *)p->userdata();
        
            // save them in Rq
            for ( int i=0; i<ch->num_pkts(); i++) {
                var = Random::uniform(0.0,1.0);
                l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
                l_NSpkt = l_data_p->NSpkt;
                
    /*            per = 1 - pow ( 1-BER, l_ch->size()*8 );*/
                per = -0.1;
                if (var <= per) // corrupted
                        push_Rq(l_NSpkt, WRONG, TAIL, i); 
                else
                        push_Rq(l_NSpkt, RIGHT, TAIL, i); 
            }
    
#ifdef TIANJI_DEBUG_8
trace_pkt("recvD 1111", pktRx_);
print_mac_queue("recvD 1111", Sq_head);
print_mac_queue("recvD 1111", Rq_head);
dump("recvD 1111");
#endif
            // free memory in data part
            remove_data_part_content(pktRx_);
    
        }    

        if ( src != index_ && dst != index_ ) {
            daemon                  = pktRx_->copy();     
        }


        /*
         * Adjust the MAC packet size - ie; strip
         * off the mac header and the mcexor header
         */
        ch->size() -= phymib_.getHdrLen11();
        ch->num_forwards() += 1;

        /*
         *  If we sent a CTS, clean up...
         */
        if(dst != MAC_BROADCAST) {
                if(size >= (int)macmib_.getRTSThreshold()) {
                        if (tx_state_ == MAC_CTS) {
                                assert(pktCTRL_);
                                Packet::free(pktCTRL_); pktCTRL_ = 0;
                                mhSend_.stop();
                                /*
                                 * Our CTS got through.
                                 */
                        } else {
                                discard(p, DROP_MAC_BUSY);
                                return;
                        }
                        tx_resume(-1);
                } else {
                        /*
                         *  We did not send a CTS and there's no
                         *  room to buffer an ACK.
                         */
//                      if(pktCTRL_) {
//                              discard(p, DROP_MAC_BUSY);
//                              return;
//                      }

#ifdef TIANJI_DEBUG_8
printf("recvData, 22222, es=%d, ed=%d, s=%d, d=%d, size=%d, cur_correct=%d, err=%d, now=%lf, in=%d\n", l_ch->prev_hop_, l_ch->next_hop_, src, dst, l_ch->size(), cur_correct_in_Rq, l_ch->error(), Scheduler::instance().clock(), index_);
#endif
                        int priority;
                        if ( dst == index_ && cur_correct_in_Rq > 0 ) {
                        // I am the dst, send MAC ACK after a SIFS if I am not rcving MAC ACK
                            get_a_frm_to_send(WRONG);
                            int l_temp = -1;
                            if ( Rq_head != NULL )
                                l_temp = Rq_head->index;
 
                            if ( HDR_CMN(daemon)->num_pkts() > 0 && l_temp != -1 ) { 
                                pktCTRL_ = daemon;
                                priority = 1;
                            } else {
                                pktCTRL_ = 0;
                                priority = -1;
                            }

#ifdef TIANJI_DEBUG_8
dump("recvData dst 111");
#endif
                            if ( mhIF_.busy() == 0 && mhSend_.busy() == 1 ) {// if collision
                                mhSend_.stop();
                                setTxState(MAC_IDLE);
                                pktTx_ = 0;
                            }

                            if ( all_acked(Sq_head) == RIGHT ) {
                                mhDefer_.stop();
                                setTxState(MAC_IDLE);
                                pktTx_ = 0;
                            }
#ifdef TIANJI_DEBUG_8
printf("recvData, dst, pri=%d, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", priority, l_ch->prev_hop_, l_ch->next_hop_, src, dst, l_ch->size(), l_ch->error(), Scheduler::instance().clock(), index_);
print_mac_queue("recvData dst", Rq_head);
print_mac_queue("recvData dst", Sq_head);
dump("recvData dst");
#endif
                        } 


                        if ( src != index_ && dst != index_ ) {
                        // I am a forwarder, send according to priority
#ifdef TIANJI_DEBUG_8
print_mac_queue("recvData forwarder", Rq_head);
#endif
                            get_a_frm_to_send(RIGHT);
                            if ( HDR_CMN(daemon)->num_pkts() > 0 ) { 
                                priority = my_priority(pktRx_); 
                                pktCTRL_ = daemon;
                            }else {
                                pktCTRL_ = 0;
                                priority = -1;
                            }

                            if ( mhIF_.busy() == 0 && mhSend_.busy() == 1 ) {// if collision
                                mhSend_.stop();
                                setTxState(MAC_IDLE);;
                                pktTx_ = 0;
                            }

#ifdef TIANJI_DEBUG_8
printf("recvData, forwarder, pri=%d, n=%d, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", priority, HDR_CMN(daemon)->num_pkts(), l_ch->prev_hop_, l_ch->next_hop_, src, dst, l_ch->size(), l_ch->error(), Scheduler::instance().clock(), index_);
dump("recvData, forwarder");
#endif
                        }


                        if(mhSend_.busy() == 0)
                                tx_resume(priority);
                }
        }
        



        struct hdr_mac_mcexor *daemon_mh = HDR_MAC_MCEXOR(daemon);
        struct hdr_cmn * daemon_ch = HDR_CMN(daemon);
        int daemon_dst = ETHER_ADDR(daemon_mh->dh_ra);
        int daemon_src = ETHER_ADDR(daemon_mh->dh_ta);

#ifdef TIANJI_DEBUG_8
printf("recvData, daemon, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, num_pkts=%d, now=%lf, in=%d\n", daemon_ch->prev_hop_, daemon_ch->next_hop_, daemon_src, daemon_dst, daemon_ch->size(), daemon_ch->error(), daemon_ch->num_pkts(), Scheduler::instance().clock(), index_);
trace_pkt("recvData daemon", daemon);
printf("recvData, 55555, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", l_ch->prev_hop_, l_ch->next_hop_, src, dst, l_ch->size(), l_ch->error(), Scheduler::instance().clock(), index_);
if ( dst == index_ || src == index_ ) {
print_mac_queue("recvData 666", Sq_head);
print_mac_queue("recvData 666", Rq_head);
}
#endif
        if ( src != index_ && dst == index_ && cur_correct_in_Rq > 0 ) {
        // i am the dst
            recv_correct_pkts( uptarget_ );
        }
#ifdef TIANJI_DEBUG_8
printf("recvData, 66666, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", l_ch->prev_hop_, l_ch->next_hop_, src, dst, l_ch->size(), l_ch->error(), Scheduler::instance().clock(), index_);
#endif
        clear_ACKED_in_Sq();
        clear_ACKED_in_Rq();
#ifdef TIANJI_DEBUG_8
print_mac_queue("recvData 777", Sq_head);
print_mac_queue("recvData 777", Rq_head);
#endif
}


void
MacMCExOR::recvACK(Packet *p)
{       
        if(tx_state_ != MAC_SEND) {
                discard(p, DROP_MAC_INVALID_STATE);
                return;
        }
        assert(pktTx_);

        mhSend_.stop();

        /*
         * The successful reception of this ACK packet implies
         * that our DATA transmission was successful.  Hence,
         * we can reset the Short/Long Retry Count and the CW.
         *
         * need to check the size of the packet we sent that's being
         * ACK'd, not the size of the ACK packet.
         */
        if((u_int32_t) HDR_CMN(pktTx_)->size() <= macmib_.getRTSThreshold())
                ssrc_ = 0;
        else
                slrc_ = 0;
        rst_cw();
        Packet::free(pktTx_); 
        pktTx_ = 0;
        
        /*
         * Backoff before sending again.
         */
        assert(mhBackoff_.busy() == 0);
        mhBackoff_.start(cw_, is_idle());

        tx_resume(-1);

        mac_log(p);
}












/**********************************************************************
 *
 *  MCExOR scheme: Tianji Li, Jul 2008, Hamilton Institute, NUIM, Ireland
 *  
 *
 **********************************************************************/





void
MacMCExOR::push_Sq(Packet *p)
{
        hdr_cmn               *ch           = HDR_CMN(p);
        struct hdr_mac_mcexor *mh           = HDR_MAC_MCEXOR(p);    
        int                   dst           = ETHER_ADDR(mh->dh_ra);
//         int                   src           = ETHER_ADDR(mh->dh_ta);
        struct mac_queue      *l_queue;

        l_queue           = (struct mac_queue *) malloc(sizeof(struct mac_queue));
        l_queue->index    = ch->uid();
        l_queue->origsize = ch->size();
        l_queue->NSpkt    = p;
        l_queue->flag     = RIGHT;
        l_queue->next     = NULL;
        l_queue->e2esrc   = index_;
        l_queue->e2edst   = dst;

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
        hdr_cmn               *ch           = HDR_CMN(p);
        struct hdr_mac_mcexor *mh           = HDR_MAC_MCEXOR(p);    
        int dst = ETHER_ADDR(mh->dh_ra);
        int src = ETHER_ADDR(mh->dh_ta);
        struct mac_queue      *l_queue;
        struct mac_queue      *temp_queue;
        int                   founded = WRONG;
 
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

#ifdef TIANJI_DEBUG_8
printf("printing in push_Rq, founded=%d, %d\n", founded, index_);
print_mac_queue("push_Rq", Rq_head);
#endif
            if ( founded == RIGHT )                 
                return;
 
           // this is a new pkt 
            l_queue           = (struct mac_queue *) malloc(sizeof(struct mac_queue));
            l_queue->index    = ch->uid();
            l_queue->origsize = ch->size();
            l_queue->NSpkt    = p;
            l_queue->flag     = flag;
            l_queue->next     = NULL;    
       
            if ( ch->next_hop_ < ch->prev_hop_ ) { // reverse direction
                l_queue->e2esrc = num_sta-1;
                l_queue->e2edst = 0;
            } else {
                l_queue->e2esrc = 0;
                l_queue->e2edst = num_sta-1;
            }

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
       
            if ( src > dst ) {
                l_queue->e2esrc = num_sta-1;
                l_queue->e2edst = index_;
            } else {
                l_queue->e2esrc = 0;
                l_queue->e2edst = index_;
            }

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
MacMCExOR::get_a_frm_to_send(int forwarding)
{
        hdr_cmn                 *ch_daemon = HDR_CMN(daemon);
        hdr_cmn                 *ch        = NULL;
        struct hdr_mac_mcexor   *dh_daemon = HDR_MAC_MCEXOR(daemon);
        struct mac_queue        *l_queue   = NULL;
        int                     daemon_size       = 0;
        int                     daemon_num_pkts   = 0;
        int                     daemon_duration   = 0;    
        int                     daemon_dst;                 
        int                     l_dir;
        int                     temp;

#ifdef TIANJI_DEBUG_8
printf("entry of get_a_frm\n");
print_mac_queue("get_a_frm Rq", Rq_head);
print_mac_queue("get_a_frm Sq", Sq_head);
#endif
        if ( forwarding == WRONG ){
             // src/dst give high pri to Rq
             if ( Rq_head != NULL && Rq_head->origsize > 0 ) {
//                      if ( Sq_head == NULL ||  all_acked(Sq_head)==RIGHT ) { // sending an MAC ACK only
                        daemon_size                 = 0;
                        Packet *l_p                 = construct_daemon(); 
                        HDR_CMN(l_p)->uid()         = -1;
                        HDR_CMN(l_p)->size()        = 0;
                        HDR_CMN(l_p)->prev_hop_     = index_;
                        if ( index_ == 0 ) 
                            HDR_CMN(l_p)->next_hop_     = num_sta -1;
                        if ( index_ == num_sta-1 ) 
                            HDR_CMN(l_p)->next_hop_     = 0;

                        push_Rq(l_p, RIGHT, HEAD, 0);
                        alloc_data_part_memory(daemon);
                        save_into_data_part(Rq_head, 0);
                        daemon_dst = Rq_head->e2edst;
                        if ( daemon_dst < index_ )
                           l_dir = REVERSE;
                        else
                           l_dir = FORWARD;
                        daemon_num_pkts = 1;
                        goto next_snd;
//                      }

//                      if (  Rq_head->flag == RIGHT ) {
//                             daemon_dst = Rq_head->e2esrc;
//                             if ( daemon_dst < index_ )
//                                l_dir = REVERSE;
//                             else
//                                l_dir = FORWARD;
//                      }
             } else if ( Sq_head == NULL ) {
                     HDR_CMN(daemon)->num_pkts() = 0;
                     clear_repeated_MAC_ACKs_in_Rq();
                     return;
             } 

// Both Rq and Sq are not empty
             l_queue   = Sq_head;

             if ( daemon_num_pkts == 0 ) 
                 alloc_data_part_memory(daemon);

             // put pkts in the frame
             temp = daemon_num_pkts;
             for ( int i=0; i<ltjmin(MAX_FRAGMENTS,cur_Sq_size); i++) {
                 ch = HDR_CMN(l_queue->NSpkt);
#ifdef TIANJI_DEBUG_8
// printf("get_a_frm, dst, i=%d, flag=%d, daemon_dst=%d, %d\n", i, l_queue->flag, l_queue->e2edst, index_);
#endif
                 if (  l_queue->flag != ACKED ) {
                        if ( daemon_num_pkts == temp ) { 
                            daemon_dst = l_queue->e2edst;
                            if ( daemon_dst < index_ )
                               l_dir = REVERSE;
                            else
                               l_dir = FORWARD;
                        }
                        // only forward pkts to a same direction
                        if ( ( l_dir == FORWARD && (daemon_dst >= l_queue->e2edst) && (l_queue->e2edst > index_) )
                             ||  ( l_dir == REVERSE && (daemon_dst <= l_queue->e2edst) && (l_queue->e2edst < index_) ) ) {
                            daemon_size += l_queue->origsize + FRAG_FCS_LEN;
                            // fragment hdrs not useful, leave blank
#ifdef TIANJI_DEBUG_8
// printf("get_a_frm, saving %d-th pkt,  daemon_size=%d, %d\n", daemon_num_pkts, daemon_size, index_);
#endif
                            save_into_data_part(l_queue, daemon_num_pkts);
                            daemon_num_pkts ++;
                            l_queue->flag = UNACKED;
                        } 
                 }
                 l_queue       = l_queue->next;
             }

next_snd:
             if ( Rq_head != NULL && Rq_head->index == -1 ) {
                 Rq_head->flag = ACKED;
                 remove_Rq_head(); //clear the MAC ACK which has to be the Rq_head.
                 cur_Rq_size --;
             }

             // ACK bitmap for data frames
             if ( Rq_head != NULL && Rq_head->index != -1 )
                 construct_ACK_bitmap(Rq_head);
             else if ( keep_ACK_bitmap == WRONG )
                 clear_ACK_bitmap();

             for ( int i=0; i<MAX_FORWARDERS; i++) {
                 dh_daemon->mcexor_hdr.ack_bitmap[i] = rq_ack_bitmap[i];
             }  
 
             ch_daemon->size()      = daemon_size + sizeof(struct hdr_mac_mcexor);
 
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
        }else if ( forwarding == RIGHT ) {
             // search Sq for more pkts if the frame is not big enough
             daemon_num_pkts = ch_daemon->num_pkts();
             daemon_dst      = ch_daemon->next_hop_;
             if ( daemon_dst < index_ )
                l_dir = REVERSE;
             else
                l_dir = FORWARD;
 
             if ( daemon_num_pkts < MAX_FRAGMENTS ) {
                 l_queue = Sq_head;
                 temp = daemon_num_pkts;
                 while ( l_queue != NULL && daemon_num_pkts < MAX_FRAGMENTS ) {
                     if ( l_queue->flag != ACKED ){ 
                        // only forward pkts to a same direction
                        if ( ( l_dir == FORWARD && daemon_dst > l_queue->e2edst )
                             ||  ( l_dir == REVERSE && daemon_dst < l_queue->e2edst ) ) {
                            daemon_size += l_queue->origsize + FRAG_FCS_LEN;
                            // fragment hdrs not useful, leave blank
                            save_into_data_part(l_queue, daemon_num_pkts);
                            daemon_num_pkts ++;
                            l_queue->flag = UNACKED;
                        } 
                     } 
                     l_queue = l_queue->next;
                 }
             }
//              ch_daemon->size()      = daemon_size + sizeof(struct hdr_mac_mcexor);
        }

/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

         // forwarder list
         if ( l_dir == FORWARD ) { // forward direction
            for ( int i=0; i<num_sta; i++) {
                if ( i <= index_ ) {
                    dh_daemon->mcexor_hdr.f_list[i] = -1;
                } else {
                    dh_daemon->mcexor_hdr.f_list[i] = i;
                } 
            }
         } else {
            for ( int i=(num_sta-1); i>=0; i--) {
                if ( i >= index_ ) {
                    dh_daemon->mcexor_hdr.f_list[i] = -1;
                } else {
                    dh_daemon->mcexor_hdr.f_list[i] = i;
                } 
            }
         }


        ch_daemon->direction() = hdr_cmn::DOWN;
        ch_daemon->txtime()    = txtime(ch_daemon->size(), dataRate_);
        ch_daemon->error()     = 0;

        dh_daemon->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
        dh_daemon->dh_fc.fc_type       = MAC_Type_Data;
        dh_daemon->dh_fc.fc_subtype    = MAC_Subtype_Data;
        dh_daemon->dh_fc.fc_to_ds      = 0;
        dh_daemon->dh_fc.fc_from_ds    = 0;
        dh_daemon->dh_fc.fc_more_frag  = 0;
        dh_daemon->dh_fc.fc_retry      = 0;
        dh_daemon->dh_fc.fc_pwr_mgt    = 0;
        dh_daemon->dh_fc.fc_more_data  = 0;
        dh_daemon->dh_fc.fc_wep        = 0;
        dh_daemon->dh_fc.fc_order      = 0;

        int temp_dst;
        if ( l_dir == FORWARD ) {// forward
            ch_daemon->prev_hop_ = 0;
            ch_daemon->next_hop_ = num_sta - 1;
            temp_dst = num_sta - 1;
        }else {
            ch_daemon->prev_hop_ = num_sta - 1;
            ch_daemon->next_hop_ = 0;
            temp_dst = 0;
        }

        temp                   = hops_to_dst(daemon);
        daemon_duration        = usec((double)temp*ch_daemon->txtime());
        dh_daemon->dh_duration = daemon_duration;
        ch_daemon->num_pkts()  = daemon_num_pkts;
#ifdef TIANJI_DEBUG_8
printf("get_a_frm, tx=%lf, n*tx=%lf, dur=%d, sec(dur)=%lf, n=%d, bitmap=%d, in=%d\n",ch_daemon->txtime(), (double)temp*ch_daemon->txtime(), dh_daemon->dh_duration, sec(dh_daemon->dh_duration), ch_daemon->num_pkts(), keep_ACK_bitmap, index_);
#endif

        STORE4BYTE(&temp_dst, (dh_daemon->dh_ra));
        STORE4BYTE(&index_, (dh_daemon->dh_ta));

#ifdef TIANJI_DEBUG_8
trace_pkt("leaving get_a",daemon);
#endif
        return ;
}


void
MacMCExOR::alloc_data_part_memory( Packet *p )
{
    if ( p->accessdata() != 0 ) 
    {
        remove_data_part_content(p);
    }
    p->allocdata(sizeof(struct data_part)*MAX_FRAGMENTS);
}

void
MacMCExOR::remove_data_part_content( Packet *p )
{
//      bzero(p->data_, datalen_);
    p->setdata(NULL);
}

void
MacMCExOR::save_into_data_part(struct mac_queue *l_queue, int whichone)
{
    // how to use data
    PacketData        *l_data;
    struct data_part  *l_data_p;

    // get the pointer
    l_data = (PacketData *)daemon->userdata();
    l_data_p = (struct data_part *)l_data->access_data(whichone*sizeof(struct data_part));

    // save information into the data part
    l_data_p->NSpkt = l_queue->NSpkt;
//     bzero(l_data_p->fcs, ETHER_FCS_LEN);
}



void
MacMCExOR::clear_ACK_bitmap( )
{
    for ( int i=0; i<MAX_FORWARDERS; i++) {
        rq_ack_bitmap[i] = rq_ack_bitmap[i] & (u_int16_t)0;
    }
}

void
MacMCExOR::construct_ACK_bitmap(struct mac_queue *q)
{
    struct mac_queue *  l_queue;
    u_int16_t           temp;
    u_int16_t           temp1 = (u_int16_t) 1;
    int                 howmany[MAX_FORWARDERS];
    int                 total_sum = 0;

    // clear the bitmap first
    clear_ACK_bitmap();

    for ( int i=0; i<MAX_FORWARDERS; i++) {
        howmany[i]       = 0;
    }

    if ( q != NULL ) {
        l_queue = Rq_head;
        while ( l_queue != NULL && total_sum < MAX_FRAGMENTS ) {
            if ( l_queue->flag == ACKED || l_queue->flag == RIGHT ) {
                temp = temp1 << howmany[l_queue->e2esrc];
                rq_ack_bitmap[l_queue->e2esrc] = rq_ack_bitmap[l_queue->e2esrc] | temp;
                total_sum ++;
            } 
            howmany[l_queue->e2esrc] ++;
            l_queue = l_queue->next;
        }
    } 

#ifdef TIANJI_DEBUG_8
    for ( int i=0; i<MAX_FORWARDERS; i++) {
        printf("MacMCExOR::construct_ACK_bitmap, rq_ack_bitmap[%d], in=%d\n", rq_ack_bitmap[i], index_);
    }
#endif
}



int
MacMCExOR::am_i_forwarder(Packet *p)
{
        struct hdr_mac_mcexor  *mh = HDR_MAC_MCEXOR(p);
        
        for ( int i=0; i<num_sta; i++) {
            if ( mh->mcexor_hdr.f_list[i] == index_ ) 
                return RIGHT;
        }
        return WRONG;
}

int
MacMCExOR::my_priority(Packet *p)
{
        struct hdr_mac_mcexor *mh = HDR_MAC_MCEXOR(p);
        int src = ETHER_ADDR(mh->dh_ta);
        int dst = ETHER_ADDR(mh->dh_ra);
        int temp;

        if ( src < dst ) { // forward direction
            temp = 1;
            for ( int i=0; i<num_sta; i++) {
                if ( mh->mcexor_hdr.f_list[i] > 0 && mh->mcexor_hdr.f_list[i] > index_ ) {
                    temp ++;
                }
            }
        } else { // reverse direction
            temp = 1;
            for ( int i=0; i<num_sta; i++) {
                if ( mh->mcexor_hdr.f_list[i] >= 0 && mh->mcexor_hdr.f_list[i] < index_ ) {
#ifdef TIANJI_DEBUG_8
printf("my_priority, f_list[%d], temp=%d, in=%d\n", i, temp, index_);
#endif
                    temp ++;
                }
            }
        }
        return temp;
}


Packet * 
MacMCExOR::construct_daemon( )
{
            Packet *p = Packet::alloc();
            hdr_cmn* ch = HDR_CMN(p);
            struct hdr_mac_mcexor *mh = HDR_MAC_MCEXOR(p);
    
            ch->uid()   = 0;
            ch->ptype() = PT_MAC;
            ch->size()  = sizeof(struct hdr_mac_mcexor);
            ch->iface() = -2;
            ch->error() = 0;
            
            mh->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
            mh->dh_fc.fc_type             = MAC_Type_Data;
            mh->dh_fc.fc_subtype          = MAC_Subtype_Data;
            mh->dh_fc.fc_to_ds            = 0;
            mh->dh_fc.fc_from_ds          = 0;
            mh->dh_fc.fc_more_frag        = 0;
            mh->dh_fc.fc_retry            = 0;
            mh->dh_fc.fc_pwr_mgt          = 0;
            mh->dh_fc.fc_more_data        = 0;
            mh->dh_fc.fc_wep              = 0;
            mh->dh_fc.fc_order            = 0;

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
            HDR_CMN(l_queue->NSpkt)->direction() = hdr_cmn::UP;
            uptarget->recv(l_queue->NSpkt, (Handler*) 0);
            remove_Rq_head( ); 
            cur_Rq_size --;
            cur_correct_in_Rq --;
        } else
            return; // only pass consecutively correct pkts to upper layer
        l_queue = l_queue->next;
    }
#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::recv_correct_pkts, %d\n", index_);
#endif
}

void
MacMCExOR::remove_Rq_head( )
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

void
MacMCExOR::remove_Sq_head( )
{
        struct mac_queue * l_queue;
        
//         if ( cur_Sq_size <= 0 ) {
//             Sq_head = NULL;
//             return;
//         }

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
MacMCExOR::update_flags_in_Sq(Packet *p)
{
    if ( Sq_head == NULL ) return;

    struct mac_queue      * l_queue = Sq_head;
    struct hdr_mac_mcexor * mh      = HDR_MAC_MCEXOR(p);
    hdr_cmn               * ch      = HDR_CMN(p);
//     int                     dst     = ETHER_ADDR(mh->dh_ra);
//     int                     src     = ETHER_ADDR(mh->dh_ta);
    u_int16_t               bitmap  = mh->mcexor_hdr.ack_bitmap[index_];  
    u_int16_t               temp1   = (u_int16_t)1;
    u_int16_t               temp;

#ifdef TIANJI_DEBUG_8
printf("MacMCExOR::update_flags_in_Sq, n=%d, cur_sq=%d, bitmap=%d, in=%d \n", ch->num_pkts(), cur_Sq_size, bitmap, index_);
#endif
    for ( u_int16_t i=0; i<cur_Sq_size; i++ ) {
        temp = temp1 << i; 
        temp = bitmap & temp;
        if ( temp > 0 && l_queue->flag == UNACKED ) {
            l_queue->flag = ACKED;
        } 
        l_queue = l_queue->next;
    }

    // clear ACKED pkts
//     clear_Sq();
}

// remove all successfully transmitted pkts in the Sq
void
MacMCExOR::clear_ACKED_in_Sq( )
{
        struct mac_queue       * l_queue = NULL;
                
        if ( cur_Sq_size == 0 )
        {
          return;
        }else if ( cur_Sq_size == 1 )
        {
                    l_queue = Sq_head;
                    if ( l_queue->flag == ACKED )
                    {
                            free(Sq_head);
                            Sq_head = NULL;
                            Sq_tail = NULL;
                            cur_Sq_size = 0;
                    }
        }else {
            while ( Sq_head != NULL && Sq_head->flag == ACKED )
            {
#ifdef TIANJI_DEBUG_8
printf("clear_ACKED_in_Sq, cur_Sq_size=%d, in=%d\n", cur_Sq_size, index_);
#endif
                remove_Sq_head();
                cur_Sq_size --;
            }
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
                            free(Rq_head);
                            Rq_head = NULL;
                            Rq_tail = NULL;
                            cur_Rq_size = 0;
                    }
        }else {
            while ( Rq_head != NULL && Rq_head->flag == ACKED )
            {
                remove_Rq_head();
                cur_Rq_size --;
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
    struct mac_queue          *l_queue;
    Packet                    *l_NSpkt = NULL;
    struct hdr_cmn            *l_ch;


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
            fprintf(stdout, "\t uid/flag/es/ed/size: %d %d %d %d %d\n", l_ch->uid(), l_queue->flag, l_ch->prev_hop_, l_ch->next_hop_, l_ch->size(), index_);
            l_queue = l_queue->next;
        }
        return;
}

int
MacMCExOR::all_acked(struct mac_queue *q)
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
MacMCExOR::all_correct(struct mac_queue *q)
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
MacMCExOR::hops_to_dst(Packet *p)
{
    hdr_cmn  * ch      = HDR_CMN(p);

    return abs( index_ - ch->next_hop_);
    
}
