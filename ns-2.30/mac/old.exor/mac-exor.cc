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
 * $Header: /tianji.cvsroot/newripple/ns-2.30/mac/exor/mac-exor.cc,v 1.3 2009/03/30 15:59:59 ripple Exp $
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

// #define TIANJI_DEBUG
// #define TIANJI_DEBUG_SHORT
// #define TIANJI_DEBUG_VERY_SHORT

#include "arp.h"
#include "ll.h"
#include "stdio.h"
#include "stdlib.h"
#include "mac-timers-exor.h"
#include "mac-exor.h"
#include "cmu-trace.h"

// Added by Sushmita to support event tracing
#include "agent.h"
#include "basetrace.h"


int hdr_mac_exor::offset_;
static class MacExORHeaderClass : public PacketHeaderClass {
public:
	MacExORHeaderClass() : PacketHeaderClass("PacketHeader/MacExOR",
                                             sizeof(hdr_mac_exor)) {
                bind_offset(&hdr_mac_exor::offset_);
        }
} class_hdr_mac_exor;


int exor_ack_frame::offset_;
static class MacExORACKHeaderClass : public PacketHeaderClass {
public:
	MacExORACKHeaderClass() : PacketHeaderClass("PacketHeader/MacExORACK",
                                             sizeof(exor_ack_frame)) {
                bind_offset(&exor_ack_frame::offset_);
        }
} class_exor_ack_frame;


inline void
MacExOR::checkBackoffTimer()
{
        if(is_idle() && mhBackoff_.paused())
                mhBackoff_.resume(phymib_.getDIFS());
        if(! is_idle() && mhBackoff_.busy() && ! mhBackoff_.paused())
                mhBackoff_.pause();
}

inline void
MacExOR::transmit(Packet *p, double timeout)
{
        tx_active_ = 1;
        struct hdr_cmn *ch = HDR_CMN(p);
        int dst, src;
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt = NULL;

#ifdef TIANJI_DEBUG_VERY_SHORT
if ( ch->ptype() != PT_MAC ) {
        struct hdr_mac_exor *dh = HDR_MAC_EXOR(p);
        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        // get the pointer of pkts in this frame
        l_data = (PacketData *)p->userdata();
        l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
        l_NSpkt = l_data_p->NSpkt;
        if ( index_ == 0 )
            printf("xmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtime=%lf, to=%lf\n", index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), txtime(p), timeout);
}
#endif

#ifdef TIANJI_DEBUG_SHORT
if ( ch->ptype() != PT_MAC ) {
        struct hdr_mac_exor *dh = HDR_MAC_EXOR(p);
        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        // get the pointer of pkts in this frame
        l_data = (PacketData *)p->userdata();
        l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
        l_NSpkt = l_data_p->NSpkt;
        if ( index_ == 0 )
            printf("\n\nxmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtime=%lf, to=%lf\n", index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), txtime(p), timeout);
        else
            printf("xmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtime=%lf, to=%lf\n", index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), txtime(p), timeout);
}else{
        struct exor_ack_frame *af = (struct exor_ack_frame*)p->access(hdr_mac::offset_);
        printf("xmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf,  txtime=%lf, to=%lf\n", index_, ch->num_pkts(), ch->uid(), ch->prev_hop_, ch->next_hop_, ETHER_ADDR(af->dh_ta), ETHER_ADDR(af->dh_ra), ch->size(), ch->error(), Scheduler::instance().clock(), txtime(p),  timeout);
}
#endif

#ifdef TIANJI_DEBUG

if ( ch->ptype() != PT_MAC ) {
        struct hdr_mac_exor *dh = HDR_MAC_EXOR(p);
        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        // get the pointer of pkts in this frame
        l_data = (PacketData *)p->userdata();
        l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
        l_NSpkt = l_data_p->NSpkt;
        printf("\n\n\nxmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf, txtime=%lf, to=%lf\n", index_, ch->num_pkts(), HDR_CMN(l_NSpkt)->uid(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), txtime(p), timeout);
        trace_pkt("xmit, p",p);
        print_mac_queue("xmit, Sq", Sq_head);
}else{
        struct exor_ack_frame *af = (struct exor_ack_frame*)p->access(hdr_mac::offset_);
        printf("\n\n\nxmit in=%d, n=%d, puid=%d, e2es/d=%d/%d, s/d=%d/%d, size=%d, err=%d, now=%lf,  txtime=%lf, to=%lf\n", index_, ch->num_pkts(), ch->uid(), ch->prev_hop_, ch->next_hop_, ETHER_ADDR(af->dh_ta), ETHER_ADDR(af->dh_ra), ch->size(), ch->error(), Scheduler::instance().clock(), txtime(p),  timeout);
        trace_pkt("xmit",p);
        print_mac_queue("xmit", Sq_head);
}

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

        if ( ch->ptype() != PT_MAC ) {
                for ( int i=0; i<HDR_CMN(p)->num_pkts(); i++) {
                    l_data = (PacketData *)p->userdata();
                    l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
                    l_NSpkt = l_data_p->NSpkt;
                    if ( l_data_p->NSpkt->ref_count() >= 0)
                        l_data_p->NSpkt->ref_count() --;
#ifdef TIANJI_DEBUG
printf("transmit, uid=%d, size=%d, refcount=%d, %d\n", HDR_CMN(p)->uid(), HDR_CMN(p)->size(), l_data_p->NSpkt->ref_count(), index_);
#endif
                }
        }

        downtarget_->recv(p->copy(), this);
        mhSend_.start(timeout);
        mhIF_.start(txtime(p));
}
inline void
MacExOR::setRxState(MacState newState)
{
        rx_state_ = newState;
        checkBackoffTimer();
}

inline void
MacExOR::setTxState(MacState newState)
{
        tx_state_ = newState;
        checkBackoffTimer();
}


/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class MacExORClass : public TclClass {
public:
        MacExORClass() : TclClass("Mac/ExOR") {}
        TclObject* create(int, const char*const*) {
        return (new MacExOR());

}
} class_mac_exor;


/* ======================================================================
   Mac  and Phy MIB Class Functions
   ====================================================================== */

PHY_MIB_EXOR::PHY_MIB_EXOR(MacExOR *parent)
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

MAC_MIB_EXOR::MAC_MIB_EXOR(MacExOR *parent)
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
MacExOR::MacExOR() :
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
        //printf("bssid in constructor %d\n",bss_id_);
        cur_Sq_size = 0;
        cur_Rq_size = 0;
        cur_correct_in_Rq = 0;

        Sq_head = NULL;
        Rq_head = NULL;

        daemon = construct_daemon();
}


int
MacExOR::command(int argc, const char*const* argv)
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

/* ======================================================================
   Debugging Routines
   ====================================================================== */
void
MacExOR::trace_pkt(char *fname, Packet *p)
{
        struct hdr_cmn            *ch       = HDR_CMN(p);
        struct hdr_mac_exor       *dh       = HDR_MAC_EXOR(p);
        struct exor_ack_frame          *ah       = HDR_EXOR_MAC_ACK(p);
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt  = NULL;
        struct hdr_cmn            *l_ch;


        fprintf(stdout,
                "%s --- (INDEX: %d, e2es/d:%d/%d, time: %2.9f)\n",
                fname, index_, ch->prev_hop_, ch->next_hop_, Scheduler::instance().clock());

        if ( dh->dh_fc.fc_type == MAC_Type_Control )
             fprintf(stdout, "\t dur/src/dst/type/size/hid/n:%d %d %d %s %d %d %d\n",
                 dh->dh_duration,
                 ETHER_ADDR(ah->dh_ta), ETHER_ADDR(ah->dh_ra),
                 packet_info.name(ch->ptype()), ch->size(), ah->highest_rcver_id, ch->num_pkts());
        else
             fprintf(stdout, "\t dur/src/dst/type/size/n:%d %d %d %s %d %d\n",
                 dh->dh_duration,
                 ETHER_ADDR(dh->dh_ta), ETHER_ADDR(dh->dh_ra),
                 packet_info.name(ch->ptype()), ch->size(), ch->num_pkts());


        fprintf(stdout, "\t f.list: %d %2d %d %d %d\n",
                dh->f_list[0],
                dh->f_list[1],
                dh->f_list[2],
                dh->f_list[3],
                dh->f_list[4]
                );

        if ( dh->dh_fc.fc_type == MAC_Type_Control ) {
            fprintf(stdout, "\t ack_bitmap: %d %d %d %d %d\n",
                    ah->ack_bitmap[0],
                    ah->ack_bitmap[1],
                    ah->ack_bitmap[2],
                    ah->ack_bitmap[3],
                    ah->ack_bitmap[4]
                    );
        }

        l_data = (PacketData *)p->userdata();
        int l_fflag;
        for ( int i=0; i<ch->num_pkts(); i++) {
                l_data_p = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
                l_NSpkt  = l_data_p->NSpkt;
                l_ch     = HDR_CMN(l_NSpkt);
                l_fflag  = l_data_p->NSpkt->fflag() ? RIGHT : WRONG;
                fprintf(stdout, "\t uid/es/ed/size/ref/fflag/p:%d %d %d %d %d %d %lx\n",
                        l_ch->uid(),
                        l_ch->prev_hop_,
                        l_ch->next_hop_,
                        l_ch->size(),
                        l_data_p->NSpkt->ref_count(),
                        l_data_p->NSpkt->fflag(),
                        (long) l_NSpkt );
        }

}

void
MacExOR::dump(char *fname)
{
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt  = NULL;
        struct hdr_cmn            *l_ch;



        fprintf(stdout,
                "%s --- (INDEX: %d, time: %2.9f)\n",
                fname, index_, Scheduler::instance().clock());

        fprintf(stdout,
                "\ttx_state_: %x, rx_state_: %x, nav: %2.9f, idle: %d\n",
                tx_state_, rx_state_, nav_, is_idle());

        if ( daemon != NULL && HDR_CMN(daemon)->num_pkts()>0 ) {
            l_data = (PacketData *)daemon->userdata();
            l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
            l_NSpkt  = l_data_p->NSpkt;
            l_ch     = HDR_CMN(l_NSpkt);
            fprintf(stdout,
                "\tTx_: %lx, Rx_: %lx, d: %lx, pkt: %lx, CTRL_: %lx, cb: %lx, ref=%d\n",
                (long) pktTx_, (long) pktRx_, (long) daemon, (long) l_NSpkt,
                (long) pktCTRL_, (long) callback_, l_NSpkt->ref_count());
        } else{
            fprintf(stdout,
                "\tTx_: %lx, Rx_: %lx, d: %lx,  CTRL_: %lx, callback: %lx\n",
                (long) pktTx_, (long) pktRx_, (long) daemon,
                (long) pktCTRL_, (long) callback_);
        }

        fprintf(stdout,
                "\tDefer: %d, Backoff: %d (%d), Recv: %d, Send: %d If: %d Nav: %d\n",
                mhDefer_.busy(), mhBackoff_.busy(), mhBackoff_.paused(),
                mhRecv_.busy(), mhSend_.busy(), mhIF_.busy(), mhNav_.busy());
        fprintf(stdout,
                "\tBackoff Expire: %f\n",
                mhBackoff_.expire());
}

void MacExOR::trace_event(char *eventtype, Packet *p)
{
        if (et_ == NULL) return;
        char *wrk = et_->buffer();
        char *nwrk = et_->nbuffer();

        struct hdr_mac_exor* dh = HDR_MAC_EXOR(p);

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
   Packet Headers Routines
   ====================================================================== */
inline int
MacExOR::hdr_dst(char* hdr, int dst )
{
        struct hdr_mac_exor *dh = (struct hdr_mac_exor*) hdr;

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
MacExOR::hdr_src(char* hdr, int src )
{
        struct hdr_mac_exor *dh = (struct hdr_mac_exor*) hdr;
        if(src > -2)
               STORE4BYTE(&src, (dh->dh_ta));
        return ETHER_ADDR(dh->dh_ta);
}

inline int
MacExOR::hdr_type(char* hdr, u_int16_t type)
{
        struct hdr_mac_exor *dh = (struct hdr_mac_exor*) hdr;
        if(type)
                STORE2BYTE(&type,(dh->dh_body));
        return GET2BYTE(dh->dh_body);
}


/* ======================================================================
   Misc Routines
   ====================================================================== */
inline int
MacExOR::is_idle()
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
MacExOR::discard(Packet *p, const char* why)
{
        hdr_mac_exor* mh = HDR_MAC_EXOR(p);
        hdr_cmn *ch = HDR_CMN(p);

        /* if the rcvd pkt contains errors, a real MAC layer couldn't
           necessarily read any data from it, so we just toss it now */
        if(ch->error() != 0) {
//                 Packet::free(p);
                p = NULL;
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
                exit(1);
        }
//         Packet::free(p);
}

void
MacExOR::capture(Packet *p)
{
        /*
         * Update the NAV so that this does not screw
         * up carrier sense.
         */
        set_nav(usec(phymib_.getEIFS() + txtime(p)));
//         Packet::free(p);
        p = NULL;
}

void
MacExOR::collision(Packet *p)
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
MacExOR::tx_resume(int priority)
{
#ifdef TIANJI_DEBUG
printf("MacExOR::tx_resume, pri=%d, %d\n", priority, index_);
#endif
        double rTime;
        assert(mhSend_.busy() == 0);
        assert(mhDefer_.busy() == 0);

        if(pktCTRL_) {
#ifdef TIANJI_DEBUG
printf("MacExOR::tx_resume, pktCTRL, %d\n", index_);
#endif
                mhDefer_.start((double)priority*phymib_.getSIFS() +
                               (double)(priority-1)*(txtime(phymib_.getACKlen(), basicRate_) + DSSS_MaxPropagationDelay) );
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
#ifdef TIANJI_DEBUG
printf("MacExOR::tx_resume, pktTx_, %d\n", index_);
#endif
start_backoff:
                if (mhBackoff_.busy() == 0) {
                        hdr_cmn *ch = HDR_CMN(pktTx_);
                        struct hdr_mac_exor *mh = HDR_MAC_EXOR(pktTx_);

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
        } else {
                // there may be more pkts available during backoff
                if ( index_ == 0 || index_ == num_sta-1)
                    get_a_frm_to_send( Sq_head );
                else
                    get_a_frm_to_send( Rq_head );

#ifdef TIANJI_DEBUG
print_mac_queue("tx_resume, Sq",Sq_head);
printf("MacExOR::tx_resume, more pkts, numpkts=%d, %d\n", HDR_CMN(daemon)->num_pkts(), index_);
#endif

                if ( HDR_CMN(daemon)->num_pkts() > 0 ) {
                    pktTx_ = daemon;
                    goto start_backoff;
                } else {
                    pktTx_ = 0;
                    if(callback_) {
                        Handler *h = callback_;
#ifdef TIANJI_DEBUG
printf("MacExOR::tx_resume, callback_, %d\n", index_);
#endif
                        h->handle((Event*) 0);
                    }
               }
        }


        setTxState(MAC_IDLE);
#ifdef TIANJI_DEBUG
dump("end of tx_resume");
#endif
}

void
MacExOR::rx_resume()
{
        assert(pktRx_ == 0);
        assert(mhRecv_.busy() == 0);
        setRxState(MAC_IDLE);
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
MacExOR::backoffHandler()
{
#ifdef TIANJI_DEBUG
printf("MacExOR::backoffHandler, %d\n", index_);
// dump("backoffHandler");
#endif

        if(pktCTRL_) {
//                 assert(mhSend_.busy() || mhDefer_.busy());
//                     return;
//                 Packet::free(pktCTRL_);
                pktCTRL_ = 0;
        }

        if(check_pktRTS() == 0)
                return;

        if(check_pktTx() == 0)
                return;
}

void
MacExOR::deferHandler()
{
#ifdef TIANJI_DEBUG
printf("MacExOR::deferHandler, now=%lf, %d\n", Scheduler::instance().clock(),index_);
#endif
        assert(pktCTRL_ || pktRTS_ || pktTx_);

        if(check_pktCTRL() == 0)
                return;
        else {
             pktCTRL_ = NULL;
        }

        assert(mhBackoff_.busy() == 0);
        if(check_pktRTS() == 0)
                return;
        if(check_pktTx() == 0)
                return;
}

void
MacExOR::navHandler()
{
#ifdef TIANJI_DEBUG
printf("MacExOR::navHandler, %d\n", index_);
dump("navHandler before");
#endif
        if( is_idle() && mhBackoff_.paused() )
                mhBackoff_.resume(phymib_.getDIFS());

//         if( is_idle() && mhBackoff_.busy()==0 && pktCTRL_ == 0 )
//             tx_resume(-1);

        // did not finish sending, resume
        if( tx_state_ == MAC_SEND && !mhSend_.busy() )
            tx_resume(-1);

//         if ( (index_==0 || index_==num_sta-1) && pktTx_!=NULL && is_idle() && !mhBackoff_.busy() && !mhSend_.busy() && !mhDefer_.busy() && !mhRecv_.busy() )
//             tx_resume(-1);

#ifdef TIANJI_DEBUG
dump("navHandler");
print_mac_queue("navHandler", Sq_head);
trace_pkt("navHandler, daemon", daemon);
#endif
}

void
MacExOR::recvHandler()
{
        recv_timer();
}

void
MacExOR::sendHandler()
{
        send_timer();
}


void
MacExOR::txHandler()
{
#ifdef TIANJI_DEBUG
printf("MacExOR::txHandler, %d\n", index_);
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
MacExOR::send_timer()
{
#ifdef TIANJI_DEBUG
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
//                 Packet::free(pktCTRL_);
                pktCTRL_ = 0;
                break;
        /*
         * Sent DATA, but did not receive an ACK packet.
         */
        case MAC_SEND:
                RetransmitDATA();
                break;
        /*
         * Sent an ACK, and now ready to resume transmission.
         */
        case MAC_ACK:
                assert(pktCTRL_);
                if ( pktCTRL_ != NULL )
//                     Packet::free(pktCTRL_);
                pktCTRL_ = 0;
                break;
        case MAC_IDLE:
                break;
        default:
                assert(0);
        }

        if ( pktCTRL_ != NULL && mhDefer_.busy()==1 ) {
        } else
            tx_resume(-1);
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
MacExOR::check_pktCTRL()
{
#ifdef TIANJI_DEBUG
printf("MacExOR::check_pktCTRL, %d\n", index_);
#endif
        struct exor_ack_frame *mh;
        double timeout;
        int src, dst;

        if(pktCTRL_ == 0)
                return -1;
        if(tx_state_ == MAC_CTS || tx_state_ == MAC_ACK || rx_state_ != MAC_IDLE)
                return -1;

        mh = HDR_EXOR_MAC_ACK(pktCTRL_);

        switch(mh->af_fc.fc_subtype) {
        case MAC_Subtype_ACK:
                setTxState(MAC_ACK);
                timeout = (double)hops_to_dst(pktCTRL_)*( txtime(phymib_.getACKlen(), basicRate_) + DSSS_MaxPropagationDelay) + (double)(hops_to_dst(pktCTRL_)-1)*phymib_.getSIFS();
                break;
        default:
                fprintf(stdout, "check_pktCTRL:Invalid MAC Control subtype\n");
                exit(1);
        }
        transmit(pktCTRL_, timeout);
        return 0;
}

int
MacExOR::check_pktRTS()
{
        struct hdr_mac_exor *mh;
        double timeout;

        assert(mhBackoff_.busy() == 0);

        if(pktRTS_ == 0)
                return -1;
        mh = HDR_MAC_EXOR(pktRTS_);

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
MacExOR::check_pktTx()
{
#ifdef TIANJI_DEBUG
printf("MacExOR::check_pktTx in=%d\n", index_);
print_mac_queue("check_pktTx",Rq_head);
#endif
        struct hdr_mac_exor *mh;
        double timeout;
        int temp;

        assert(mhBackoff_.busy() == 0);

        // there may be more pkts available during backoff
        if ( index_ == 0 || index_ == num_sta-1)
            get_a_frm_to_send( Sq_head );
        else
            get_a_frm_to_send( Rq_head );

        if ( HDR_CMN(daemon)->num_pkts() > 0 )
            pktTx_ = daemon;
        else
            pktTx_ = 0;

        if(pktTx_ == 0)
                return -1;

        mh = HDR_MAC_EXOR(pktTx_);
        STORE4BYTE(&index_, (mh->dh_ta));
        HDR_CMN(pktTx_)->direction() = hdr_cmn::DOWN;;
        update_forwarding_list(HDR_CMN(pktTx_)->next_hop_, pktTx_, MAC_Type_Data);

#ifdef TIANJI_DEBUG
printf("MacExOR::check_pktTx, %d\n", index_);
trace_pkt("check_pktTx", pktTx_);
print_mac_queue("check_pktTx", Sq_head);
dump("check_pktTx");
#endif

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
                        timeout = (double)(temp+1)*DSSS_MaxPropagationDelay
                               + temp*phymib_.getSIFS()
                               + temp*txtime(phymib_.getACKlen(), basicRate_)
                               + sec(mh->dh_duration);
               } else
                        timeout = txtime(pktTx_);
                break;
        default:
                fprintf(stderr, "check_pktTx:Invalid MAC Control subtype\n");
                exit(1);
        }

        if ( pktCTRL_ ) {
//             Packet::free( pktCTRL_ );
            pktCTRL_ = 0;
        }


        transmit(pktTx_, timeout);
        return 0;
}
/*
 * Low-level transmit functions that actually place the packet onto
 * the channel.
 */
void
MacExOR::sendRTS(int dst)
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
MacExOR::sendCTS(int dst, double rts_duration)
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
MacExOR::sendACK(int dst, Packet *l_pktRx)
{
        Packet *p            = Packet::alloc();
        hdr_cmn* ch          = HDR_CMN(p);
        struct exor_ack_frame *af = (struct exor_ack_frame*)p->access(hdr_mac::offset_);
        int temp = index_;

        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt = NULL;
        l_data = (PacketData *)l_pktRx->userdata();
        l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
        l_NSpkt = l_data_p->NSpkt;

        assert(pktCTRL_ == 0);

        bzero(af, MAC_HDR_LEN);
        ch->uid()                     = 0;
        ch->ptype()                   = PT_MAC;
        af->highest_rcver_id          = index_;
        ch->size()                    = phymib_.getACKlen();
        ch->txtime()                  = txtime(ch->size(), basicRate_);
        ch->iface()                   = -2;
        ch->error()                   = 0;
        af->af_fc.fc_protocol_version = MAC_ProtocolVersion;
        af->af_fc.fc_type             = MAC_Type_Control;
        af->af_fc.fc_subtype          = MAC_Subtype_ACK;
        af->af_fc.fc_to_ds            = 0;
        af->af_fc.fc_from_ds          = 0;
        af->af_fc.fc_more_frag        = 0;
        af->af_fc.fc_retry            = 0;
        af->af_fc.fc_pwr_mgt          = 0;
        af->af_fc.fc_more_data        = 0;
        af->af_fc.fc_wep              = 0;
        af->af_fc.fc_order            = 0;
        af->af_duration               = 0;
        for ( int i=0; i<MAX_FORWARDERS; i++) {
            af->ack_bitmap[i] = rq_ack_bitmap[i];
        }

        fill_forwarding_list ( dst, p, MAC_Type_Control );

        STORE4BYTE(&dst,    (af->dh_ra));
        STORE4BYTE(&temp, (af->dh_ta));

        ch->prev_hop_ = index_;
        ch->next_hop_ = dst;

        af->pID       = HDR_CMN(l_NSpkt)->uid();

        pktCTRL_ = p;
}

void
MacExOR::sendDATA(Packet *p)
{
        assert(pktTx_ == 0);

        if ( daemon!=NULL )
        {
                pktTx_ = daemon;
        }else
        {
                printf("MacExOR::sendDATA, no frame to send, quit...%d\n",index_);
                exit(0);
        }

}

/* ======================================================================
   Retransmission Routines
   ====================================================================== */
void
MacExOR::RetransmitRTS()
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
MacExOR::RetransmitDATA()
{
        struct hdr_cmn *ch;
        struct hdr_mac_exor *mh;
        u_int32_t *rcount, thresh;
        assert(mhBackoff_.busy() == 0);

        assert(pktTx_);
        assert(pktRTS_ == 0);

        ch = HDR_CMN(pktTx_);
        mh = HDR_MAC_EXOR(pktTx_);

#ifdef TIANJI_DEBUG
print_mac_queue("Retx Rq", Rq_head);
if ( pktTx_ != NULL ) trace_pkt("Retx pktTx_", pktTx_);
#endif
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

        (*rcount)++;

#ifdef TIANJI_DEBUG
printf("Retx count=%d, uid=%d, in=%d\n", *rcount, ch->uid(), index_);
#endif
        if(*rcount >= thresh) {
                *rcount = 0;
                rst_cw();
        }
        else {
                inc_cw();
        }
        mhBackoff_.start(cw_, is_idle());

        return;
}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
MacExOR::send(Packet *p, Handler *h)
{
        double rTime;
        struct hdr_mac_exor* dh = HDR_MAC_EXOR(p);

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
MacExOR::recv(Packet *p, Handler *h)
{
        struct hdr_cmn        *hdr   = HDR_CMN(p);
        int src, dst;
        struct exor_ack_frame      *af = HDR_EXOR_MAC_ACK(p);
        struct hdr_mac_exor   *mh = HDR_MAC_EXOR(p);

        if ( hdr->ptype() == PT_MAC) {
            dst    = ETHER_ADDR(af->dh_ra);
            src    = ETHER_ADDR(af->dh_ta);
        } else {
            dst    = ETHER_ADDR(mh->dh_ra);
            src    = ETHER_ADDR(mh->dh_ta);
        }
        int e2esrc = hdr->prev_hop_;
        int e2edst = hdr->next_hop_;

        assert(initialized());

        if(hdr->direction() == hdr_cmn::DOWN) {


#ifdef TIANJI_DEBUG
printf("recv() down, uid=%d, size=%d, now=%lf, refcount=%d, in=%d\n", hdr->uid(), hdr->size(), Scheduler::instance().clock(), p->ref_count(), index_);
#endif

#ifdef TIANJI_DEBUG
trace_pkt("recv down1", daemon);
print_mac_queue("recv down1", Sq_head);
printf("recv() down, uid=%d, size=%d, now=%lf, refcount=%d, in=%d\n", hdr->uid(), hdr->size(), Scheduler::instance().clock(), p->ref_count(), index_);
// dump("recv down");
#endif
                if ( cur_Sq_size < MAX_Sq_SIZE ) {
                     push_Sq(p->refcopy());
                } else {
                     discard(p, DROP_RTR_QFULL);
                }

                daemon_handler = h;
                callback_      = h;

                get_a_frm_to_send( Sq_head );
                send(daemon, daemon_handler);

                if ( cur_Sq_size < MAX_Sq_SIZE ) {
                    // fetch another pkt if any from the upper layer
                    Handler *h_temp = h;
                    h = 0;
                    h_temp->handle((Event*) 0);
                }
                return;
        }





#ifdef TIANJI_DEBUG
printf("recv() up before, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, in ---------- %d\n", hdr->prev_hop_, hdr->next_hop_, src, dst, hdr->size(), hdr->error(),  index_);
dump("recv up before");
trace_pkt("recv up before", p);
print_mac_queue("recv up before", Sq_head);
#endif




        if(tx_active_ && hdr->error() == 0) {
                hdr->error() = 1;
        }



// forwarders should cancel their tx if hear high priority MAC ACKs
        if ( (0 != index_) && (num_sta-1 != index_) && hops_to_dst(p) < abs(e2esrc-e2edst)
             && hdr->ptype() == PT_MAC ) {
            // this MAC ACK is for me
            if ( af->ack_bitmap[index_] > 0 ) {
//                 pktTx_ = 0;
                pktCTRL_ = 0;
            }

//             if(mhBackoff_.busy())
//                     mhBackoff_.stop();
            if(mhDefer_.busy())
                    mhDefer_.stop();
        }

        // got a data when defering to send an MAC ACK
        if ( pktCTRL_ != NULL && mhDefer_.busy()==1 && hdr->ptype()==PT_TCP ) {
#ifdef TIANJI_DEBUG
printf("latest change is here.....%d\n", index_);
#endif
            hdr->error() = 1;
        }

        if ( am_i_forwarder(p) == WRONG ) {
            hdr->error() = 1;
        }

        if ( src < dst && src > index_ ) { // fd: hear a high priority data
            hdr->error() = 1;
        }

        if ( src > dst && src < index_ ) { // rd: hear a high priority data
            hdr->error() = 1;
        }

        if ( e2esrc > num_sta || e2edst > num_sta ) { // htp from 802.11
            hdr->error() = 1;
        }

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

#ifdef TIANJI_DEBUG_SHORT
printf("recv() up before, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, in ---------- %d\n", hdr->prev_hop_, hdr->next_hop_, src, dst, hdr->size(), hdr->error(),  index_);
#endif

#ifdef TIANJI_DEBUG
printf("recv() up, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, in ---------------- %d\n", hdr->prev_hop_, hdr->next_hop_, src, dst, hdr->size(), hdr->error(), index_);
dump("recv up");
trace_pkt("recv up", p);
print_mac_queue("recv up", Sq_head);
#endif
}

void
MacExOR::recv_timer()
{
        hdr_cmn *ch      = HDR_CMN(pktRx_);
        hdr_mac_exor *mh = HDR_MAC_EXOR(pktRx_);
        exor_ack_frame    *ah = HDR_EXOR_MAC_ACK(pktRx_);

        u_int8_t  type = mh->dh_fc.fc_type;
        u_int8_t  subtype = mh->dh_fc.fc_subtype;

        assert(pktRx_);
        assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);


        int dst, src;
        if ( type == MAC_Type_Control ) {
            dst = ETHER_ADDR(ah->dh_ra);
            src = ETHER_ADDR(ah->dh_ta);
        } else {
            dst = ETHER_ADDR(mh->dh_ra);
            src = ETHER_ADDR(mh->dh_ta);
        }

#ifdef TIANJI_DEBUG
printf("recv_timer, n=%d, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in ------ %d\n", ch->num_pkts(), ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), index_);
print_mac_queue("recv_timer, Rq", Rq_head);
print_mac_queue("recv_timer. Sq", Sq_head);
dump("recv_timer");
if ( daemon != NULL)
   trace_pkt("recv_timer, daemon", daemon);
#endif


        if(tx_active_) {
#ifdef TIANJI_DEBUG
printf("recv_timer 111111111 %d\n", index_);
#endif
//                 Packet::free(pktRx_);
                goto done;
        }


        if(rx_state_ == MAC_COLL) {
#ifdef TIANJI_DEBUG
printf("recv_timer 2222222222 %d\n", index_);
#endif
                discard(pktRx_, DROP_MAC_COLLISION);
                set_nav(usec(phymib_.getEIFS()));
                goto done;
        }


        if( ch->error() == 1 ) {
//                 Packet::free(pktRx_);
                set_nav(usec(phymib_.getEIFS()));
#ifdef TIANJI_DEBUG
printf("recv_timer 3333333, EIFS=%lf, in=%d\n", phymib_.getEIFS(), index_);
printf("recv_timer 33333333 %d\n", index_);
print_mac_queue("recv_timer 33333",Sq_head);
#endif
                goto done;
        }


        /* tap out - */
        if (tap_ && type == MAC_Type_Data &&
            MAC_Subtype_Data == subtype )
                tap_->tap(pktRx_);
        if (netif_->node()->energy_model() &&
            netif_->node()->energy_model()->adaptivefidelity()) {
                src = ETHER_ADDR(mh->dh_ta);
                netif_->node()->energy_model()->add_neighbor(src);
        }



#ifdef TIANJI_DEBUG
printf("recv_timer, uid=%d, size=%d, type=%x, subtype=%x, %d\n", ch->uid(), ch->size(), type, subtype, index_);
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
MacExOR::recvRTS(Packet *p)
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
MacExOR::txtime(Packet *p)
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
MacExOR::txtime(double psz, double drt)
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
MacExOR::recvCTS(Packet *p)
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
MacExOR::recvDATA(Packet *p)
{
        struct hdr_mac_exor *dh   = HDR_MAC_EXOR(p);
        struct hdr_cmn      *ch   = HDR_CMN(p);
        struct hdr_cmn      *f_ch = HDR_CMN(daemon);
        struct hdr_mac_exor *f_mh = HDR_MAC_EXOR(daemon);
        int dst, src, size, priority, l_dir;

        dst = ETHER_ADDR(dh->dh_ra);
        src = ETHER_ADDR(dh->dh_ta);
        size = ch->size();

        double                    per;
        double                    var;
        PacketData                *l_data;
        struct data_part          *l_data_p;
        Packet                    *l_NSpkt = NULL;

#ifdef TIANJI_DEBUG
printf("recvData, 11111, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), index_);
trace_pkt("recvD 1111", p);
print_mac_queue("recvD 1111", Sq_head);
print_mac_queue("recvD 1111", Rq_head);
dump("recvD 1111");
#endif
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
                        push_Rq(l_NSpkt, WRONG, TAIL, src);
                else
                        push_Rq(l_NSpkt, RIGHT, TAIL, src);
            }

#ifdef TIANJI_DEBUG
print_mac_queue("recvD nnnnn", Rq_head);
#endif
        last_rx_end_time = Scheduler::instance().clock();

#ifdef TIANJI_DEBUG
print_mac_queue("recvD 2222 Rq", Rq_head);
trace_pkt("recvD 2222 daemon", daemon);
#endif

        ch->num_forwards() += 1;

        /*
         *  If we sent a CTS, clean up...
         */
        if(dst != MAC_BROADCAST) {
                if(size >= (int)macmib_.getRTSThreshold()) {
                     printf("recvData, never smaller than threshold!\n");
                     exit(0);
                } else {
                        if ( pktCTRL_ != NULL && mhDefer_.busy() == 1 ) {
                        // do nothing if started sending MAC ACK
                        } else {
                            construct_ACK_bitmap(Rq_head, p);
                            sendACK(src, p);
                            priority = hops_to_dst(p) + 1;
                            if( mhSend_.busy() == 0 )
                                tx_resume(priority);
                            else {
                                mhSend_.stop();
                                tx_resume(priority);
                            }
                        }

                }
        }

        if ( src != index_ && dst == index_ && cur_correct_in_Rq > 0 ) {
            // i am the dst
            recv_correct_pkts( uptarget_ );
        }


#ifdef TIANJI_DEBUG
printf("recvData, 66666, pri=%d, es=%d, ed=%d, s=%d, d=%d, size=%d, err=%d, now=%lf, in=%d\n", priority, ch->prev_hop_, ch->next_hop_, src, dst, ch->size(), ch->error(), Scheduler::instance().clock(), index_);
print_mac_queue("recvData 66666", Sq_head);
print_mac_queue("recvData 66666", Rq_head);
#endif


        if ( dst == index_ ) clear_RIGHT_in_Rq();



#ifdef TIANJI_DEBUG
print_mac_queue("recvData 77777", Sq_head);
print_mac_queue("recvData 77777", Rq_head);
dump("recvData 77777");
#endif
}


void
MacExOR::recvACK(Packet *p)
{
        struct exor_ack_frame *dh      = HDR_EXOR_MAC_ACK(p);
        struct hdr_cmn *ch        = HDR_CMN(p);
        int e2edst, e2esrc, src, priority, dst;

        e2edst = ch->next_hop_;
        e2esrc = ch->prev_hop_;
        src = ETHER_ADDR(dh->dh_ta);
        dst = ETHER_ADDR(dh->dh_ra);

        if ( (src<dst && index_ == num_sta-1)  || (src>dst && index_ == 0) ) {
#ifdef TIANJI_DEBUG
print_mac_queue("recvACK 111", Sq_head);
#endif
            update_flags_in_Sq(p);
#ifdef TIANJI_DEBUG
print_mac_queue("recvACK 222", Sq_head);
#endif
            clear_ACKED_in_Sq(p);
#ifdef TIANJI_DEBUG
print_mac_queue("recvACK 333", Sq_head);
if ( daemon != NULL ) trace_pkt("recvACK 333",daemon);
#endif
        }

        if ( 0 != index_ && num_sta-1 != index_ ) { // forwarders
#ifdef TIANJI_DEBUG
print_mac_queue("recvACK 111 forwarders", Rq_head);
#endif
            update_flags_in_Rq(p);
#ifdef TIANJI_DEBUG
print_mac_queue("recvACK 222 forwarders", Rq_head);
#endif
            clear_ACKED_in_Rq();
#ifdef TIANJI_DEBUG
print_mac_queue("recvACK 333 forwarders", Rq_head);
if ( daemon != NULL ) trace_pkt("recvACK 333 forwarders",daemon);
#endif
        }


        if ( e2esrc == index_ ||
             e2edst == index_ ||
             abs(e2edst-src) < abs(e2edst-index_)  ) {
           // i am the src/dst or a high priority forwarder
            if( tx_state_ != MAC_SEND ) {
                    discard(p, DROP_MAC_INVALID_STATE);
                    return;
            }

            ssrc_ = 0;
            slrc_ = 0;

            rst_cw();

            remove_data_part_content(daemon);
            HDR_CMN(daemon)->num_pkts() = 0;
            HDR_CMN(daemon)->size() = 0;
            pktTx_ = 0;

//             Packet::free(p);
            p = 0;

            mhSend_.stop();

            tx_resume(-1);
      } else { // i should forward this heard ACK
#ifdef TIANJI_DEBUG
if (p!=NULL) trace_pkt("recvACK, p",p);
if (pktCTRL_!=NULL) trace_pkt("recvACK, pktCTRL_",pktCTRL_);
#endif
            priority = abs(index_ - src) ;
            pktCTRL_ = p;
            dh       = HDR_EXOR_MAC_ACK(pktCTRL_);
            ch       = HDR_CMN(pktCTRL_);
            STORE4BYTE(&index_, (dh->dh_ta));
            ch->direction() = hdr_cmn::DOWN;
            ch->error()     = 0;

            fill_forwarding_list (e2edst, p, MAC_Type_Control);




            // high priority forwarders have got this pkt, I can rest
//             if ( pktTx_ != NULL && mhDefer_.busy() ) {
            if ( pktTx_ != NULL && is_this_ACK_for_me(p) == RIGHT ) {
                pktTx_ = NULL;
                if ( HDR_CMN(daemon)->num_pkts() > 0 ) {
                    HDR_CMN(daemon)->num_pkts() = 0;
                    HDR_CMN(daemon)->size()     = 0;
                }
            }


            if ( mhSend_.busy() ) mhSend_.stop();
            tx_resume(priority);
      }
}












/**********************************************************************
 *
 *  ExOR scheme: Tianji Li, Jul 2008, Hamilton Institute, NUIM, Ireland
 *
 *
 **********************************************************************/





void
MacExOR::push_Sq(Packet *p)
{
        hdr_cmn               *ch           = HDR_CMN(p);
        struct hdr_mac_exor   *mh           = HDR_MAC_EXOR(p);
        int                   dst           = ETHER_ADDR(mh->dh_ra);
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
MacExOR::push_Rq(Packet *p, int flag, int head_or_tail, int local_src)
{
        hdr_cmn               *ch             = HDR_CMN(p);
        struct hdr_mac_exor   *mh             = HDR_MAC_EXOR(p);
        int                   dst             = ETHER_ADDR(mh->dh_ra);
        int                   src             = ETHER_ADDR(mh->dh_ta);
        struct mac_queue      *l_queue;
        struct mac_queue      *temp_queue;
        int                   founded = WRONG;

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
                l_queue->e2esrc = local_src;
                l_queue->e2edst = index_;
            } else {
                l_queue->e2esrc = local_src;
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
MacExOR::get_a_frm_to_send( mac_queue *q)
{
        hdr_cmn                 *ch_daemon = HDR_CMN(daemon);
        hdr_cmn                 *ch        = NULL;
        struct hdr_mac_exor     *dh_daemon = HDR_MAC_EXOR(daemon);
        struct mac_queue        *l_queue   = NULL;
        int                     daemon_size       = 0;
        int                     daemon_num_pkts   = ch_daemon->num_pkts();
        int                     daemon_duration   = 0;
        int                     daemon_dst        = -1;
        int                     l_dir;
        int                     temp;

        if ( q == NULL ) {
#ifdef TIANJI_DEBUG
printf("get_a_frm, this queue is null, return! %d\n", index_);
trace_pkt("get_a, daemon", daemon);
#endif
            return;
        }


#ifdef TIANJI_DEBUG
printf("entry of get_a_frm, daemon_num_pkts=%d, %d\n", daemon_num_pkts, index_);
dump("entering get_a");
trace_pkt("entering get_a",daemon);
print_mac_queue("get_a_frm Rq", Rq_head);
print_mac_queue("get_a_frm Sq", Sq_head);
#endif

         l_queue   = q;

         if ( daemon_num_pkts==0 )
             alloc_data_part_memory(daemon);

         if ( daemon_num_pkts>0 ) {
             daemon_dst  = ch_daemon->next_hop_;
             daemon_size = ch_daemon->size() - sizeof(struct hdr_mac_exor);
             if ( daemon_dst < index_ )
                l_dir = REVERSE;
             else
                l_dir = FORWARD;
         }

         int which_queue_size;
         if ( q == Sq_head ) which_queue_size = cur_Sq_size;
         if ( q == Rq_head ) which_queue_size = cur_Rq_size;

         // put pkts in the frame
         temp = daemon_num_pkts;
         for ( int i=daemon_num_pkts; i<daemon_num_pkts+ltjmin(MAX_FRAGMENTS-daemon_num_pkts,which_queue_size); i++) {
             ch = HDR_CMN(l_queue->NSpkt);
             if (  l_queue->flag != ACKED ) {
                    if ( daemon_num_pkts == temp && daemon_dst==-1 ) {
                        daemon_dst = ch->next_hop_;
                    }
                    if ( daemon_dst < index_ )
                       l_dir = REVERSE;
                    else
                       l_dir = FORWARD;

                    // only forward pkts to a same direction
                    if ( ( l_dir == FORWARD && (daemon_dst >= l_queue->e2edst) && (daemon_dst > index_) )
                     ||  ( l_dir == REVERSE && (daemon_dst <= l_queue->e2edst) && (daemon_dst < index_) ) ) {
                        daemon_size += l_queue->origsize + FRAG_FCS_LEN;
                        // fragment hdrs not useful, leave blank
#ifdef TIANJI_DEBUG
printf("get_a_frm, saving %d-th pkt,  daemon_size=%d, %d\n", daemon_num_pkts, daemon_size, index_);
#endif
                        save_into_data_part(l_queue, daemon_num_pkts);
                        daemon_num_pkts ++;
                        l_queue->flag = UNACKED;
                    }
             }
             l_queue       = l_queue->next;
         }

        fill_forwarding_list (daemon_dst, daemon, MAC_Type_Data);

        ch_daemon->size()      = daemon_size + sizeof(struct hdr_mac_exor);
        ch_daemon->direction() = hdr_cmn::DOWN;
        ch_daemon->txtime()    = txtime(ch_daemon->size(), dataRate_);
        ch_daemon->error()     = 0;
        ch_daemon->ptype()     = PT_TCP;

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
        daemon_duration        = usec(ch_daemon->txtime());
        dh_daemon->dh_duration = daemon_duration;
        ch_daemon->num_pkts()  = daemon_num_pkts;
        STORE4BYTE(&temp_dst, (dh_daemon->dh_ra));
        STORE4BYTE(&index_, (dh_daemon->dh_ta));

#ifdef TIANJI_DEBUG
printf("get_a_frm, tx=%lf, n*tx=%lf, dur=%d, sec(dur)=%lf, in=%d\n",ch_daemon->txtime(), (double)temp*ch_daemon->txtime(), dh_daemon->dh_duration, sec(dh_daemon->dh_duration), index_);
trace_pkt("leaving get_a",daemon);
dump("leaving get_a");
#endif
        return ;
}


void
MacExOR::alloc_data_part_memory( Packet *p )
{
    if ( p->accessdata() != 0 )
    {
        remove_data_part_content(p);
    }
    p->allocdata(sizeof(struct data_part)*MAX_FRAGMENTS);
}

void
MacExOR::remove_data_part_content( Packet *p )
{
    struct hdr_cmn    * ch = HDR_CMN(p);
    PacketData        *l_data;
    struct data_part  *l_data_p;

#ifdef TIANJI_DEBUG
printf("remove_data_part in=%d\n", index_);
#endif

//     l_data = (PacketData *)p->userdata();
//     for ( int i=0; i<ch->num_pkts(); i++) {
//         l_data_p = (struct data_part *)l_data->access_data(i*sizeof(struct data_part));
// #ifdef TIANJI_DEBUG
// printf("remove_data_part, uid=%d, refcount=%d, size=%d, %d\n", HDR_CMN(l_data_p->NSpkt)->uid(), l_data_p->NSpkt->ref_count(), HDR_CMN(l_data_p->NSpkt)->size(), index_ );
// #endif
//     }

    p->setdata(NULL);
    ch->num_pkts() = 0;
}

void
MacExOR::save_into_data_part(struct mac_queue *l_queue, int whichone)
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
MacExOR::clear_ACK_bitmap( )
{
    for ( int i=0; i<MAX_FORWARDERS; i++) {
        rq_ack_bitmap[i] = rq_ack_bitmap[i] & (u_int16_t)0;
    }
}

void
MacExOR::construct_ACK_bitmap(struct mac_queue *q, Packet *l_pktRx)
{
    struct mac_queue *  l_queue;
    u_int16_t           temp;
    u_int16_t           temp1 = (u_int16_t) 1;
    int                 howmany[MAX_FORWARDERS];
    int                 total_sum = 0;

    PacketData                *l_data;
    struct data_part          *l_data_p;
    Packet                    *l_NSpkt = NULL;

    l_data   = (PacketData *)l_pktRx->userdata();
    l_data_p = (struct data_part *)l_data->access_data( 0 * sizeof(struct data_part) );
    l_NSpkt  = l_data_p->NSpkt;


#ifdef TIANJI_DEBUG
print_mac_queue("construct_ACK_bitmap", Rq_head);
#endif

    // clear the bitmap first
    clear_ACK_bitmap();

    for ( int i=0; i<MAX_FORWARDERS; i++) {
        howmany[i]       = 0;
    }

    if ( q != NULL ) {
        l_queue = Rq_head;
        while ( l_queue != NULL && total_sum < MAX_FRAGMENTS ) {
                if ( (l_queue->flag == RIGHT || l_queue->flag == UNACKED ) && HDR_CMN(l_queue->NSpkt)->uid()==HDR_CMN(l_NSpkt)->uid() ) {
                    temp = temp1 << howmany[l_queue->e2esrc];
                    rq_ack_bitmap[l_queue->e2esrc] = rq_ack_bitmap[l_queue->e2esrc] | temp;
                    total_sum ++;
                    howmany[l_queue->e2esrc] ++;
                }
                l_queue = l_queue->next;
        }
    }

#ifdef TIANJI_DEBUG
    for ( int i=0; i<MAX_FORWARDERS; i++) {
        printf("MacExOR::construct_ACK_bitmap, rq_ack_bitmap[%d], in=%d\n", rq_ack_bitmap[i], index_);
    }
#endif
}


void
MacExOR::update_forwarding_list(int dst, Packet *p, u_int8_t type)
{
#ifdef TIANJI_DEBUG
printf("update_forwarding_list, dst=%d, type=%d, %d\n", dst, type, index_);
#endif
     if ( type == MAC_Type_Data ) { // data frames
         hdr_mac_exor   *mh  = HDR_MAC_EXOR(p);
         for ( int i=0; i<MAX_FORWARDERS; i++) {
              mh->f_list[i] = -1;
         }

         if ( index_ < dst ) { // forward direction
            for ( int i=index_+1; i<=dst; i++) {
                mh->f_list[i] = i;
            }
         }

         if ( index_ > dst ) { // reverse direction
            for ( int i=dst; i<index_; i++) {
                mh->f_list[i] = i;
            }
         }
    }


     if ( type == MAC_Type_Control ) { // MAC ACK
         exor_ack_frame      *ah  = HDR_EXOR_MAC_ACK(p);
         for ( int i=0; i<MAX_FORWARDERS; i++) {
              ah->f_list[i] = -1;
         }

         if ( index_ < dst ) { // forward direction
            for ( int i=index_+1; i<=dst; i++) {
                ah->f_list[i] = i;
            }
         }

         if ( index_ > dst ) { // reverse direction
            for ( int i=dst; i<index_; i++) {
                ah->f_list[i] = i;
            }
         }
    }

}



int
MacExOR::am_i_forwarder(Packet *p)
{
        struct hdr_cmn  *ch = HDR_CMN(p);

        if ( ch->ptype() == PT_TCP ) {
            struct hdr_mac_exor  *mh = HDR_MAC_EXOR(p);

            for ( int i=0; i<num_sta; i++) {
                if ( mh->f_list[i] == index_ )
                    return RIGHT;
            }
        }

        if ( ch->ptype() == PT_MAC ) {
            struct exor_ack_frame  *ah = HDR_EXOR_MAC_ACK(p);

            for ( int i=0; i<num_sta; i++) {
                if ( ah->f_list[i] == index_ )
                    return RIGHT;
            }
        }

        return WRONG;
}


Packet *
MacExOR::construct_daemon( )
{
            Packet *p = Packet::alloc();
            hdr_cmn* ch = HDR_CMN(p);
            struct hdr_mac_exor *mh = HDR_MAC_EXOR(p);

            ch->uid()   = 0;
            ch->ptype() = PT_TCP;
            ch->size()  = sizeof(struct hdr_mac_exor);
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
printf("construct_daemon, %p, in=%d\n", p, index_);
            return p;
}


int
MacExOR::ltjmin(int a, int b)
{
    if ( a < b )
        return a;
    return b;
}


void
MacExOR::recv_correct_pkts(NsObject *uptarget)
{
    struct mac_queue *     l_queue;

    l_queue = Rq_head;
    while ( l_queue != NULL ){
        if ( l_queue->flag == RIGHT ) {
            HDR_CMN(l_queue->NSpkt)->direction() = hdr_cmn::UP;
            HDR_CMN(l_queue->NSpkt)->size() = HDR_CMN(l_queue->NSpkt)->size();
//             l_queue->NSpkt->ref_count() --;
#ifdef TIANJI_DEBUG
printf("MacExOR::recv_correct_pkts, uid=%d, size=%d, refcount=%d, %d\n", HDR_CMN(l_queue->NSpkt)->uid(), HDR_CMN(l_queue->NSpkt)->size(), l_queue->NSpkt->ref_count(), index_);
#endif

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
MacExOR::remove_Rq_head( )
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
MacExOR::remove_Sq_head( )
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
MacExOR::update_flags_in_Sq(Packet *p)
{
    if ( Sq_head == NULL ) return;

    struct mac_queue      * l_queue = Sq_head;
    struct exor_ack_frame      * ah      = HDR_EXOR_MAC_ACK(p);
    u_int16_t               bitmap  = ah->ack_bitmap[index_];
    u_int16_t               temp1   = (u_int16_t)1;
    u_int16_t               temp;/**/


#ifdef TIANJI_DEBUG
printf("MacExOR::update_flags_in_Sq,  cur_sq=%d, bitmap=%d, in=%d \n", cur_Sq_size, bitmap, index_);
#endif
    for ( u_int16_t i=0; i<cur_Sq_size; i++ ) {
        if ( HDR_EXOR_MAC_ACK(p)->pID == HDR_CMN(l_queue->NSpkt)->uid() ) {
            temp = temp1 << i;
            temp = bitmap & temp;
            if ( temp > 0 && l_queue->flag == UNACKED ) {
                l_queue->flag = ACKED;
            }
        }
        l_queue = l_queue->next;
    }

}



void
MacExOR::update_flags_in_Rq(Packet *p)
{
    if ( Rq_head == NULL ) return;

    struct mac_queue      * l_queue = Rq_head;
    struct exor_ack_frame      * ah      = HDR_EXOR_MAC_ACK(p);
    u_int16_t             bitmap  ;
    u_int16_t             temp1   = (u_int16_t)1;
    u_int16_t             temp, howmany=0;

    bitmap  = ah->ack_bitmap[HDR_CMN(p)->next_hop_];

#ifdef TIANJI_DEBUG
printf("MacExOR::update_flags_in_Rq,  ackpID=%d, datauid=%d, bitmap=%d, in=%d \n",  HDR_EXOR_MAC_ACK(p)->pID, HDR_CMN(l_queue->NSpkt)->uid(), bitmap, index_);
#endif

//         struct hdr_mac_exor *dh = HDR_MAC_EXOR(p);
//         dst = ETHER_ADDR(dh->dh_ra);
    for ( u_int16_t i=0; i<cur_Rq_size; i++ ) {
        if ( HDR_EXOR_MAC_ACK(p)->pID == HDR_CMN(l_queue->NSpkt)->uid() ) {
            temp = temp1 << howmany;
            temp = bitmap & temp;
            if ( temp > 0 && l_queue->flag != ACKED ) {
                l_queue->flag = ACKED;
            }
            howmany ++;
        }
        l_queue = l_queue->next;
    }

}

// remove all successfully transmitted pkts in the Sq
void
MacExOR::clear_ACKED_in_Sq( Packet *p)
{
#ifdef TIANJI_DEBUG
printf("clear_ACKED_in_Sq, cur_Sq_size=%d, in=%d\n", cur_Sq_size, index_);
#endif
// #ifdef TIANJI_DEBUG_SHORT
// printf("clear_ACKED_in_Sq, cur_Sq_size=%d, in=%d\n", cur_Sq_size, index_);
// #endif
        struct mac_queue       * l_queue = NULL;
        struct exor_ack_frame        *af      = (struct exor_ack_frame*)p->access(hdr_mac::offset_);


        if ( cur_Sq_size == 0 )
        {
          return;
        }else if ( cur_Sq_size == 1 )
        {
                    l_queue = Sq_head;
                    if ( l_queue->flag == ACKED && l_queue->index ==  af->pID )
                    {
                            free(Sq_head);
                            Sq_head = NULL;
                            Sq_tail = NULL;
                            cur_Sq_size = 0;
                    }
        }else {
            while ( Sq_head != NULL && Sq_head->flag == ACKED && Sq_head->index ==  af->pID )
            {
                remove_Sq_head();
                cur_Sq_size --;
            }
        }


//                             if ( pktTx_ != NULL ) pktTx_ = NULL;
//                             if ( HDR_CMN(daemon)->num_pkts() > 0 ) {
//                                 HDR_CMN(daemon)->num_pkts() = 0;
//                                 remove_data_part_content(daemon);
//                             }


        return;
}



// remove all successfully transmitted pkts in the Rq
void
MacExOR::clear_ACKED_in_Rq( )
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
#ifdef TIANJI_DEBUG
printf("clear_ACKED_in_Rq, cur_Rq_size=%d, in=%d\n", cur_Rq_size, index_);
#endif
                remove_Rq_head();
                cur_Rq_size --;
            }
        }
        return;
}



// remove all successfully transmitted pkts in the Rq
void
MacExOR::clear_RIGHT_in_Rq( )
{
        struct mac_queue       * l_queue = NULL;

        if ( cur_Rq_size == 0 )
        {
          return;
        }else if ( cur_Rq_size == 1 )
        {
                    l_queue = Rq_head;
                    if ( l_queue->flag == RIGHT )
                    {
                            free(Rq_head);
                            Rq_head = NULL;
                            Rq_tail = NULL;
                            cur_Rq_size = 0;
                    }
        }else {
            while ( Rq_head != NULL && Rq_head->flag == RIGHT )
            {
                remove_Rq_head();
                cur_Rq_size --;
            }
        }
        return;
}


// remove all pkts in the Rq
void
MacExOR::clear_ALL_in_Rq( )
{
        struct mac_queue       * l_queue = NULL;

        if ( cur_Rq_size == 0 )
        {
          return;
        }else if ( cur_Rq_size == 1 )
        {
                    l_queue = Rq_head;
                    {
                            free(Rq_head);
                            Rq_head = NULL;
                            Rq_tail = NULL;
                            cur_Rq_size = 0;
                    }
        }else {
            while ( Rq_head != NULL )
            {
                remove_Rq_head();
                cur_Rq_size --;
            }
        }
        return;
}


// remove all successfully transmitted pkts in the Rq
void
MacExOR::clear_repeated_MAC_ACKs_in_Rq( )
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
MacExOR::print_mac_queue(char *fname, struct mac_queue *q)
{
    struct mac_queue          *l_queue;
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
            fprintf(stdout, "\t uid/flag/es/ed/size: %d %d %d %d %d %lx, in=%d\n", l_ch->uid(), l_queue->flag, l_ch->prev_hop_, l_ch->next_hop_, l_ch->size(), (long) l_queue->NSpkt,index_);
            l_queue = l_queue->next;
        }
        return;
}

int
MacExOR::all_acked(struct mac_queue *q)
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
MacExOR::all_correct(struct mac_queue *q)
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
MacExOR::hops_to_dst(Packet *p)
{
    hdr_cmn  * ch      = HDR_CMN(p);
    int temp = abs( index_ - ch->next_hop_);
#ifdef TIANJI_DEBUG
printf("hops_to_dst, %d, in=%d\n",temp, index_);
#endif
    return temp;
}

void
MacExOR::fill_forwarding_list(int dst, Packet *p, u_int8_t type)
{
     if ( type == MAC_Type_Data ) { // data frames
         hdr_mac_exor   *mh  = HDR_MAC_EXOR(p);
         for ( int i=0; i<MAX_FORWARDERS; i++) {
              mh->f_list[i] = -1;
         }

         if ( index_ < dst ) { // forward direction
            for ( int i=index_+1; i<=dst; i++) {
                mh->f_list[i] = i;
            }
         }

         if ( index_ > dst ) { // reverse direction
            for ( int i=dst; i<index_; i++) {
                mh->f_list[i] = i;
            }
         }
    }


     if ( type == MAC_Type_Control ) { // MAC ACK
         exor_ack_frame      *ah  = HDR_EXOR_MAC_ACK(p);
         for ( int i=0; i<MAX_FORWARDERS; i++) {
              ah->f_list[i] = -1;
         }

         if ( index_ < dst ) { // forward direction
            for ( int i=index_+1; i<=dst; i++) {
                ah->f_list[i] = i;
            }
         }

         if ( index_ > dst ) { // reverse direction
            for ( int i=dst; i<index_; i++) {
                ah->f_list[i] = i;
            }
         }
    }

}


int
MacExOR::is_this_ACK_for_me(Packet *p)
{

     double correct_time      = (double)(num_sta-1) * txtime(p) + (double)(num_sta-1) * (phymib_.getSIFS()+ DSSS_MaxPropagationDelay);

// printf("is_this_ACK, corr_time=%lf, diff=%lf, in=%d\n", correct_time, Scheduler::instance().clock() - last_rx_end_time, index_);

     if ( Scheduler::instance().clock() - last_rx_end_time  > correct_time)
         return WRONG;
     else
         return RIGHT;
}
