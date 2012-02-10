/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Tianji Li, Jul. 06, 2008.
 */

#ifndef ns_mac_afr_h
#define ns_mac_afr_h

// Added by Sushmita to support event tracing (singal@nunki.usc.edu)
#include "address.h"
#include "ip.h"
#include "packet.h"

#include "mac-timers-afr.h"
#include "marshall.h"
#include <math.h>
#include <stddef.h>
#include <mac-802_11.h>

struct afr_data_hdr {
        struct fragment_hdr    frag_hdr[MAX_FRAGMENTS];
};

struct afr_ack_hdr {
        int                    pID;
        u_int16_t              FCS;
};

struct afr_mac_data_hdr {
        struct frame_control    dh_fc;
        int                     dh_duration;
        u_char                  dh_ra[ETHER_ADDR_LEN];
        u_char                  dh_ta[ETHER_ADDR_LEN];
        u_char                  dh_3a[ETHER_ADDR_LEN];
        u_int16_t               dh_scontrol;
        struct afr_data_hdr     afr_data_hdr;
        u_char                  dh_body[1]; // size of 1 for ANSI compatibility

        // Header access methods
        static int offset_;
        inline static int& offset() { return offset_; }
        inline static afr_mac_data_hdr* access(const Packet* p) {
                return (afr_mac_data_hdr*) p->access(offset_);
        }
};


struct afr_mac_ack_hdr {
        struct frame_control    dh_fc;
        int                     dh_duration;
        u_char                  dh_ra[ETHER_ADDR_LEN];
        u_char                  dh_ta[ETHER_ADDR_LEN];
        u_char                  dh_3a[ETHER_ADDR_LEN];
        u_int16_t               dh_scontrol;
        struct afr_ack_hdr      afr_ack_hdr;
        u_char                  dh_body[1]; // size of 1 for ANSI compatibility

        // Header access methods
        static int offset_;
        inline static int& offset() { return offset_; }
        inline static afr_mac_ack_hdr* access(const Packet* p) {
                return (afr_mac_ack_hdr*) p->access(offset_);
        }

};





class PHY_MIB_AFR {
public:
        PHY_MIB_AFR(MacAFR *parent);

        inline u_int32_t getCWMin() { return(CWMin); }
        inline u_int32_t getCWMax() { return(CWMax); }
        inline double getSlotTime() { return(SlotTime); }
        inline double getSIFS() { return(SIFSTime); }
        inline double getPIFS() { return(SIFSTime + SlotTime); }
        inline double getDIFS() { return(SIFSTime + 2 * SlotTime); }

        double basicRate;
        inline double getEIFS() {
                return(SIFSTime + PHYheader + (double)(22+8*getACKlen()) / basicRate);
        }
        inline u_int32_t getACKlen() {
            return ( sizeof(struct frame_control) +
                     sizeof(struct afr_ack_hdr) +
                     sizeof(u_char)*ETHER_ADDR_LEN*3 +
                     sizeof(int) +
                     sizeof(u_int16_t) +
                     sizeof(u_char)  );
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


class MAC_MIB_AFR {
public:

        MAC_MIB_AFR(MacAFR *parent);

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

class MacAFR : public Mac {
        friend class    DeferTimerAFR;
        friend class    BackoffTimerAFR;
        friend class    IFTimerAFR;
        friend class    NavTimerAFR;
        friend class    RxTimerAFR;
        friend class    TxTimerAFR;
public:
        MacAFR();
        void            recv(Packet *p, Handler *h);
        inline int      hdr_dst(char* hdr, int dst = -2);
        inline int      hdr_src(char* hdr, int src = -2);
        inline int      hdr_type(char* hdr, u_int16_t type = 0);
        inline int      bss_id() { return bss_id_; }
        void            trace_event(char *, Packet *);
        EventTrace *    et_;

        Packet                  *data_daemon; // used at MAC, never freed
        Packet                  *ack_daemon;  // used at MAC, never freed
        Packet                  *p_copy_for_tx;
        Handler                 *daemon_handler;
        double                  BER;
        double                  num_sta;
        double                  MAX_Sq_SIZE;
        struct mac_queue *      Sq_head;
        struct mac_queue *      Sq_tail;
        struct mac_queue *      Rq_head;
        struct mac_queue *      Rq_tail;
        struct mac_queue *      temp_Sq_head;
        struct mac_queue *      temp_Rq_head;
        int                     cur_Sq_size;
        int                     cur_Rq_size;
        int                     cur_correct_in_Rq;


        void      push_Sq(Packet *p);
        void      push_Rq(Packet *p, int flag, int head_or_tail );
        void      update_data_daemon_to_send( );
        void      update_ack_daemon_to_send( Packet *p, int rcv_array[MAX_FRAGMENTS]);
        void      remove_data_part_content( Packet *p );
        void      alloc_data_part_memory( Packet *p );
        void      save_into_data_part (struct mac_queue *l_queue, int whichone);
        void      construct_ACK_pID( Packet *p, int rcv_array[MAX_FRAGMENTS]);
        int       rq_ack_pID;
        Packet *  reconstruct_data_daemon_free_NSpkts();
        Packet *  reconstruct_data_daemon();
        Packet *  reconstruct_ack_daemon();
        Packet *  construct_data_daemon();
        Packet *  construct_ack_daemon();
        int       ltjmin(int a, int b);
        void      recv_correct_pkts(NsObject *uptarget);
        void      remove_Rq_head( );
        void      remove_Sq_head( );
        void      update_flags_in_Sq(Packet *p); // by src/dst
        void      clear_ACKED_in_Sq( );
        void      clear_failed_in_Sq( int howmany);
        void      clear_ACKED_in_Rq( );
        void      clear_repeated_MAC_ACKs_in_Rq( );
        void      print_mac_queue(char *fname, struct mac_queue *q);

        inline int mac_ack_size () {
                return ( sizeof(struct frame_control) +
                         sizeof(struct afr_ack_hdr) +
                         sizeof(u_char)*ETHER_ADDR_LEN*3 +
                         sizeof(int) +
                         sizeof(u_int16_t) +
                         sizeof(u_char)  );
        }
        inline int mac_data_size () {
                return ( sizeof(struct frame_control) +
                         sizeof(struct afr_data_hdr) +
                         sizeof(u_char)*ETHER_ADDR_LEN*3 +
                         sizeof(int) +
                         sizeof(u_int16_t) +
                         sizeof(u_char)   );
        }


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
        void            sendDATA(Packet *p);
        void            RetransmitRTS();
        void            RetransmitDATA();

        void            recvDATA(Packet *p);

        void            capture(Packet *p);
        void            collision(Packet *p);
        void            discard(Packet *p, const char* why);
        void            rx_resume(void);
        void            tx_resume( );

        inline int      is_idle(void);
public:
        void            trace_pkt(char* fname, Packet *p);
private:
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
        PHY_MIB_AFR         phymib_;
        MAC_MIB_AFR         macmib_;
       int     bss_id_;
       enum    {IBSS_ID=MAC_BROADCAST};


private:
        double          basicRate_;
        double          dataRate_;

        IFTimerAFR           mhIF_;          // interface timer
        NavTimerAFR          mhNav_;         // NAV timer
        RxTimerAFR           mhRecv_;        // incoming packets
        TxTimerAFR           mhSend_;        // outgoing packets

        DeferTimerAFR        mhDefer_;       // defer timer
        BackoffTimerAFR      mhBackoff_;     // backoff timer

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





#endif /* __mac_802_afr__ */

