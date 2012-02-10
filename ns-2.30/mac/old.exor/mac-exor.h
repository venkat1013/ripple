/* -*-  Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Tianji Li, Jul. 06, 2008.
 */

#ifndef ns_mac_exor_h
#define ns_mac_exor_h

// Added by Sushmita to support event tracing (singal@nunki.usc.edu)
#include "address.h"
#include "ip.h"
#include "queue.h"
#include "packet.h"

#include "stdlib.h"


#include "mac-timers-exor.h"
#include "marshall.h"
#include <math.h>
#include <stddef.h>
#include <mac-802_11.h>


// *************************** data structrues *************************************

struct exor_ack_frame {
        struct frame_control    af_fc;
        u_int16_t               af_duration;
        u_char                  dh_ra[ETHER_ADDR_LEN];
        u_char                  dh_ta[ETHER_ADDR_LEN];
        u_char                  af_fcs[ETHER_FCS_LEN];
        u_int16_t               ack_bitmap[MAX_FORWARDERS];
        int                     highest_rcver_id;
        int                     pID;
        int                     f_list[MAX_FORWARDERS];
        u_int16_t               space_holder[MAX_FORWARDERS]; // f_list + space_holder = 6*MAX_FORWARDERS

        // Header access methods
        static int offset_;
        inline static int& offset() { return offset_; }
        inline static exor_ack_frame* access(const Packet* p) {
                return (exor_ack_frame*) p->access(offset_);
        }
};

struct hdr_mac_exor {
        struct frame_control    dh_fc;
        int                     dh_duration;
        u_char                  dh_ra[ETHER_ADDR_LEN];
        u_char                  dh_ta[ETHER_ADDR_LEN];
        u_char                  dh_3a[ETHER_ADDR_LEN];
        u_int16_t               dh_scontrol;
        u_char                  dh_body[1]; // size of 1 for ANSI compatibility
        int                     f_list[MAX_FORWARDERS];
        u_int16_t               space_holder[MAX_FORWARDERS]; // f_list + space_holder = 6*MAX_FORWARDERS
        struct fragment_hdr     frag_hdr[MAX_FRAGMENTS];

        // Header access methods
        static int offset_;
        inline static int& offset() { return offset_; }
        inline static hdr_mac_exor* access(const Packet* p) {
                return (hdr_mac_exor*) p->access(offset_);
        }
};



class PHY_MIB_EXOR {
public:
        PHY_MIB_EXOR(MacExOR *parent);

        inline u_int32_t getCWMin() { return(CWMin); }
        inline u_int32_t getCWMax() { return(CWMax); }
        inline double getSlotTime() { return(SlotTime); }
        inline double getSIFS() { return(SIFSTime); }
        inline double getPIFS() { return(SIFSTime + SlotTime); }
        inline double getDIFS() { return(SIFSTime + 2 * SlotTime); }

        double basicRate;

        inline double getEIFS() {
printf("getEIFS, aaa=%lf\n", (double)(22.0+8.0*getACKlen()) / basicRate);
              return(2.0*(SIFSTime + PHYheader + (double)(22.0+8.0*getACKlen()) / basicRate + DSSS_MaxPropagationDelay));
        }
        // 802.11a sends phy hdr within a fixed duration, no need to consider PLCPhdr anymore
        inline u_int32_t getHdrLen11() {
                return(sizeof(struct hdr_mac_exor) + ETHER_FCS_LEN);
        }
       inline u_int32_t getRTSlen() {
               return(sizeof(struct rts_frame));
       }
       inline u_int32_t getCTSlen() {
               return(sizeof(struct cts_frame));
       }
       inline u_int32_t getACKlen() {
               return(sizeof(struct exor_ack_frame));
       }


 private:

        u_int32_t       CWMin;
        u_int32_t       CWMax;
        double          SlotTime;
        double          SIFSTime;
        u_int32_t       PreambleLength;
        u_int32_t       PLCPHeaderLength;
        double          PLCPDataRate;
};


class MAC_MIB_EXOR {
public:

        MAC_MIB_EXOR(MacExOR *parent);

private:
        u_int32_t       RTSThreshold;
        u_int32_t       ShortRetryLimit;
        u_int32_t       LongRetryLimit;
public:
        u_int32_t       FailedCount;
        u_int32_t       RTSFailureCount;
        u_int32_t       ACKFailureCount;
 public:
       inline u_int32_t getRTSThreshold() { return(RTSThreshold);}
       inline u_int32_t getShortRetryLimit() { return(ShortRetryLimit);}
       inline u_int32_t getLongRetryLimit() { return(LongRetryLimit);}
};



// *************************** the MAC class *************************************

class MacExOR : public Mac {
        friend class    DeferTimerExOR;
        friend class    BackoffTimerExOR;
        friend class    IFTimerExOR;
        friend class    NavTimerExOR;
        friend class    RxTimerExOR;
        friend class    TxTimerExOR;
public:
        MacExOR();
        void            recv(Packet *p, Handler *h);
        inline int      hdr_dst(char* hdr, int dst = -2);
        inline int      hdr_src(char* hdr, int src = -2);
        inline int      hdr_type(char* hdr, u_int16_t type = 0);
        inline int      bss_id() { return bss_id_; }
        void            trace_event(char *, Packet *);
        EventTrace *    et_;

        Packet                  *daemon; // used at MAC, never freed
        Handler                 *daemon_handler;
        double                  BER;
        int                     num_sta;
        double                  MAX_Sq_SIZE;
        u_int16_t               rq_ack_bitmap[MAX_FORWARDERS];
        struct mac_queue *      Sq_head;
        struct mac_queue *      Sq_tail;
        struct mac_queue *      Rq_head;
        struct mac_queue *      Rq_tail;
        struct mac_queue *      temp_Sq_head;
        struct mac_queue *      temp_Rq_head;
        int                     cur_Sq_size;
        int                     cur_Rq_size;
        int                     cur_correct_in_Rq;
        int                     keep_ACK_bitmap;
        double                  last_rx_end_time;

        void      push_Sq(Packet *p);
        void      push_Rq(Packet *p, int flag, int head_or_tail, int local_src);
        void      get_a_frm_to_send( mac_queue *q);
        void      remove_data_part_content( Packet *p );
        void      alloc_data_part_memory( Packet *p );
        void      save_into_data_part (struct mac_queue *l_queue, int whichone);
        void      construct_ACK_bitmap(struct mac_queue *q, Packet *l_pktRx);
        int       am_i_forwarder(Packet *p);
        Packet *  construct_daemon();
        int       ltjmin(int a, int b);
        void      recv_correct_pkts(NsObject *uptarget);
        void      remove_Rq_head( );
        void      remove_Sq_head( );
        void      update_flags_in_Sq(Packet *p); // by src/dst
        void      update_flags_in_Rq(Packet *p); // by forwarders
        void      clear_ACKED_in_Sq(Packet *p);
        void      clear_ACKED_in_Rq( );
        void      clear_RIGHT_in_Rq( );
        void      clear_ALL_in_Rq( );
        void      clear_repeated_MAC_ACKs_in_Rq( );
        void      update_flags_in_Rq_Sq(Packet *p); // by forwarders
        void      print_mac_queue(char *fname, struct mac_queue *q);
        int       all_acked(struct mac_queue *q);
        int       all_correct(struct mac_queue *q);
        int       hops_to_dst(Packet *p);
        void      clear_ACK_bitmap( );
        void      fill_forwarding_list(int dst, Packet *p, u_int8_t type);
        void      update_forwarding_list(int dst, Packet *p, u_int8_t type);
        int       is_this_ACK_for_me(Packet *p);

protected:
        void    backoffHandler(void);
        void    deferHandler(void);
        void    navHandler(void);
        void    recvHandler(void);
        void    sendHandler(void);
        void    txHandler(void);

private:
        int             command(int argc, const char*const* argv);
        int             bugFix_timer_;
        void            recv_timer(void);
        void            send_timer(void);
        int             check_pktCTRL();
        int             check_pktRTS();
        int             check_pktTx();

        void            send(Packet *p, Handler *h);
        void            sendRTS(int dst);
        void            sendCTS(int dst, double duration);
        void            sendACK(int dst, Packet *l_pktRx);
        void            sendDATA(Packet *p);
        void            RetransmitRTS();
        void            RetransmitDATA();

        void            recvRTS(Packet *p);
        void            recvCTS(Packet *p);
        void            recvACK(Packet *p);
        void            recvDATA(Packet *p);

        void            capture(Packet *p);
        void            collision(Packet *p);
        void            discard(Packet *p, const char* why);
        void            rx_resume(void);
        void            tx_resume(int);

        inline int      is_idle(void);
        void            trace_pkt(char* fname, Packet *p);
        void            dump(char* fname);

        inline int initialized() {
                return (cache_ && logtarget_
                        && Mac::initialized());
        }

        inline void mac_log(Packet *p) {
                logtarget_->recv(p, (Handler*) 0);
        }

        double txtime(Packet *p);
        double txtime(double psz, double drt);
        double txtime(int bytes) { /* clobber inherited txtime() */ abort(); return 0;}

        inline void transmit(Packet *p, double timeout);
        inline void checkBackoffTimer(void);
        inline void postBackoff(int pri);
        inline void setRxState(MacState newState);
        inline void setTxState(MacState newState);


        inline void inc_cw() {
                cw_ = (cw_ << 1) + 1;
                if(cw_ > phymib_.getCWMax())
                        cw_ = phymib_.getCWMax();
        }
        inline void rst_cw() { cw_ = phymib_.getCWMin(); }

        inline double sec(double t) { return(t *= 1.0e-6); }
        inline int usec(double t) {
                int  us = (int)floor((t *= 1e6) + 0.5);
                return us;
        }
/*        inline double sec(double t) { return(t *= 1.0e-6); }
        inline u_int16_t usec(double t) {
                u_int16_t us = (u_int16_t)floor((t *= 1e6) + 0.5);
                return us;
        }*/
        inline void set_nav(u_int16_t us) {
                double now = Scheduler::instance().clock();
                double t = us * 1e-6;
                if((now + t) > nav_) {
                        nav_ = now + t;
                        if(mhNav_.busy())
                                mhNav_.stop();
                        mhNav_.start(t);
                }
        }

protected:
        PHY_MIB_EXOR         phymib_;
        MAC_MIB_EXOR         macmib_;
       int     bss_id_;
       enum    {IBSS_ID=MAC_BROADCAST};


private:
        double          basicRate_;
        double          dataRate_;

        IFTimerExOR           mhIF_;          // interface timer
        NavTimerExOR          mhNav_;         // NAV timer
        RxTimerExOR           mhRecv_;        // incoming packets
        TxTimerExOR           mhSend_;        // outgoing packets

        DeferTimerExOR        mhDefer_;       // defer timer
        BackoffTimerExOR      mhBackoff_;     // backoff timer

        double          nav_;           // Network Allocation Vector

        MacState        rx_state_;      // incoming state (MAC_RECV or MAC_IDLE)
        MacState        tx_state_;      // outgoint state
        int             tx_active_;     // transmitter is ACTIVE

        Packet          *eotPacket_;    // copy for eot callback

        Packet          *pktRTS_;       // outgoing RTS packet
        Packet          *pktCTRL_;      // outgoing non-RTS packet

        u_int32_t       cw_;            // Contention Window
        u_int32_t       ssrc_;          // STA Short Retry Count
        u_int32_t       slrc_;          // STA Long Retry Count

        int             min_frame_len_;

        NsObject*       logtarget_;
        NsObject*       EOTtarget_;     // given a copy of packet at TX end

        u_int16_t       sta_seqno_;     // next seqno that I'll use
        int             cache_node_count_;
        Host            *cache_;
};





#endif /* __mac_802_exor__ */

