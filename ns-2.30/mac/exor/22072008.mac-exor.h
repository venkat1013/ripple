/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Tianji Li, Jul. 06, 2008.
 */

#ifndef ns_mac_exor_h
#define ns_mac_exor_h

// Added by Sushmita to support event tracing (singal@nunki.usc.edu)
#include "address.h"
#include "ip.h"

#ifdef TIANJI_EXOR
#include "stdlib.h"
#define PHYheader                       0.000020        // 20 us
#define RIGHT                           1
#define WRONG                           0
#endif

#include "mac-timers-exor.h"
#include "marshall.h"
#include <math.h>
#include <stddef.h>

class EventTrace;

#define GET_ETHER_TYPE(x)		GET2BYTE((x))
#define SET_ETHER_TYPE(x,y)            {u_int16_t t = (y); STORE2BYTE(x,&t);}

/* ======================================================================
   Frame Formats
   ====================================================================== */

#define	MAC_ProtocolVersion	0x00

#define MAC_Type_Management	0x00
#define MAC_Type_Control	0x01
#define MAC_Type_Data		0x02
#define MAC_Type_Reserved	0x03

#define MAC_Subtype_RTS		0x0B
#define MAC_Subtype_CTS		0x0C
#define MAC_Subtype_ACK		0x0D
#define MAC_Subtype_Data	0x00


struct frame_control {
	u_char		fc_subtype		: 4;
	u_char		fc_type			: 2;
	u_char		fc_protocol_version	: 2;

	u_char		fc_order		: 1;
	u_char		fc_wep			: 1;
	u_char		fc_more_data		: 1;
	u_char		fc_pwr_mgt		: 1;
	u_char		fc_retry		: 1;
	u_char		fc_more_frag		: 1;
	u_char		fc_from_ds		: 1;
	u_char		fc_to_ds		: 1;
};

struct rts_frame {
	struct frame_control	rf_fc;
	u_int16_t		rf_duration;
	u_char			rf_ra[ETHER_ADDR_LEN];
	u_char			rf_ta[ETHER_ADDR_LEN];
	u_char			rf_fcs[ETHER_FCS_LEN];
};

struct cts_frame {
	struct frame_control	cf_fc;
	u_int16_t		cf_duration;
	u_char			cf_ra[ETHER_ADDR_LEN];
	u_char			cf_fcs[ETHER_FCS_LEN];
};

struct ack_frame {
	struct frame_control	af_fc;
	u_int16_t		af_duration;
	u_char			af_ra[ETHER_ADDR_LEN];
	u_char			af_fcs[ETHER_FCS_LEN];
};

// XXX This header does not have its header access function because it shares
// the same header space with hdr_mac.
struct hdr_mac_exor {
	struct frame_control	dh_fc;
	u_int16_t		dh_duration;
	u_char                  dh_ra[ETHER_ADDR_LEN];
        u_char                  dh_ta[ETHER_ADDR_LEN];
        u_char                  dh_3a[ETHER_ADDR_LEN];
	u_int16_t		dh_scontrol;
	u_char			dh_body[1]; // size of 1 for ANSI compatibility
};


/* ======================================================================
   Definitions
   ====================================================================== */

/* Must account for propagation delays added by the channel model when
 * calculating tx timeouts (as set in tcl/lan/ns-mac.tcl).
 *   -- Gavin Holland, March 2002
 */
#define DSSS_MaxPropagationDelay        0.000002        // 2us   XXXX

class PHY_MIB {
public:
	PHY_MIB(MacExOR *parent);

	inline u_int32_t getCWMin() { return(CWMin); }
        inline u_int32_t getCWMax() { return(CWMax); }
	inline double getSlotTime() { return(SlotTime); }
	inline double getSIFS() { return(SIFSTime); }
	inline double getPIFS() { return(SIFSTime + SlotTime); }
	inline double getDIFS() { return(SIFSTime + 2 * SlotTime); }
#ifdef TIANJI_EXOR
        double basicRate;
        inline double getEIFS() {
              return(SIFSTime + PHYheader + (double)(22+8*getACKlen()) / basicRate);
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
               return(sizeof(struct ack_frame));
       }
#else
        inline double getEIFS() {
                // see (802.11-1999, 9.2.10)
                return(SIFSTime + getDIFS()
                       + (8 *  getACKlen())/PLCPDataRate);
        }

	inline u_int32_t getPreambleLength() { return(PreambleLength); }
	inline double getPLCPDataRate() { return(PLCPDataRate); }
	
	inline u_int32_t getPLCPhdrLen() {
		return((PreambleLength + PLCPHeaderLength) >> 3);
	}

	inline u_int32_t getHdrLen11() {
		return(getPLCPhdrLen() + offsetof(struct hdr_mac_exor, dh_body[0])
                       + ETHER_FCS_LEN);
	}

	inline u_int32_t getRTSlen() {
		return(getPLCPhdrLen() + sizeof(struct rts_frame));
	}
	
	inline u_int32_t getCTSlen() {
		return(getPLCPhdrLen() + sizeof(struct cts_frame));
	}
	
	inline u_int32_t getACKlen() {
		return(getPLCPhdrLen() + sizeof(struct ack_frame));
	}
#endif

 private:




	u_int32_t	CWMin;
	u_int32_t	CWMax;
	double		SlotTime;
	double		SIFSTime;
	u_int32_t	PreambleLength;
	u_int32_t	PLCPHeaderLength;
	double		PLCPDataRate;
};


/*
 * IEEE 802.11 Spec, section 11.4.4.2
 *      - default values for the MAC Attributes
 */
#define MAC_FragmentationThreshold	2346		// bytes
#define MAC_MaxTransmitMSDULifetime	512		// time units
#define MAC_MaxReceiveLifetime		512		// time units

class MAC_MIB {
public:

	MAC_MIB(MacExOR *parent);

private:
	u_int32_t	RTSThreshold;
	u_int32_t	ShortRetryLimit;
	u_int32_t	LongRetryLimit;
public:
	u_int32_t	FailedCount;	
	u_int32_t	RTSFailureCount;
	u_int32_t	ACKFailureCount;
 public:
       inline u_int32_t getRTSThreshold() { return(RTSThreshold);}
       inline u_int32_t getShortRetryLimit() { return(ShortRetryLimit);}
       inline u_int32_t getLongRetryLimit() { return(LongRetryLimit);}
};


/* ======================================================================
   The following destination class is used for duplicate detection.
   ====================================================================== */
class Host {
public:
	LIST_ENTRY(Host) link;
	u_int32_t	index;
	u_int32_t	seqno;
};


/* ======================================================================
   The actual 802.11 MAC class.
   ====================================================================== */
class MacExOR : public Mac {
	friend class DeferTimerExOR;


	friend class BackoffTimerExOR;
	friend class IFTimerExOR;
	friend class NavTimerExOR;
	friend class RxTimerExOR;
	friend class TxTimerExOR;
public:
	MacExOR();
	void		recv(Packet *p, Handler *h);
	inline int	hdr_dst(char* hdr, int dst = -2);
	inline int	hdr_src(char* hdr, int src = -2);
	inline int	hdr_type(char* hdr, u_int16_t type = 0);
	
	inline int bss_id() { return bss_id_; }
	
	// Added by Sushmita to support event tracing
        void trace_event(char *, Packet *);
        EventTrace *et_;
#ifdef TIANJI_EXOR
        double  BER;
#endif

protected:
	void	backoffHandler(void);
	void	deferHandler(void);
	void	navHandler(void);
	void	recvHandler(void);
	void	sendHandler(void);
	void	txHandler(void);

private:
	int		command(int argc, const char*const* argv);

	/* In support of bug fix described at
	 * http://www.dei.unipd.it/wdyn/?IDsezione=2435	 
	 */
	int bugFix_timer_;

	/*
	 * Called by the timers.
	 */
	void		recv_timer(void);
	void		send_timer(void);
	int		check_pktCTRL();
	int		check_pktRTS();
	int		check_pktTx();

	/*
	 * Packet Transmission Functions.
	 */
	void	send(Packet *p, Handler *h);
	void 	sendRTS(int dst);
	void	sendCTS(int dst, double duration);
	void	sendACK(int dst);
	void	sendDATA(Packet *p);
	void	RetransmitRTS();
	void	RetransmitDATA();

	/*
	 * Packet Reception Functions.
	 */
	void	recvRTS(Packet *p);
	void	recvCTS(Packet *p);
	void	recvACK(Packet *p);
	void	recvDATA(Packet *p);

	void		capture(Packet *p);
	void		collision(Packet *p);
	void		discard(Packet *p, const char* why);
	void		rx_resume(void);
	void		tx_resume(void);

	inline int	is_idle(void);

	/*
	 * Debugging Functions.
	 */
	void		trace_pkt(Packet *p);
	void		dump(char* fname);

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
	inline u_int16_t usec(double t) {
		u_int16_t us = (u_int16_t)floor((t *= 1e6) + 0.5);
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
	PHY_MIB         phymib_;
        MAC_MIB         macmib_;

       /* the macaddr of my AP in BSS mode; for IBSS mode
        * this is set to a reserved value IBSS_ID - the
        * MAC_BROADCAST reserved value can be used for this
        * purpose
        */
       int     bss_id_;
       enum    {IBSS_ID=MAC_BROADCAST};


private:
	double		basicRate_;
 	double		dataRate_;
	
	/*
	 * Mac Timers
	 */
	IFTimerExOR		mhIF_;		// interface timer
	NavTimerExOR	mhNav_;		// NAV timer
	RxTimerExOR		mhRecv_;		// incoming packets
	TxTimerExOR		mhSend_;		// outgoing packets

	DeferTimerExOR	mhDefer_;	// defer timer
	BackoffTimerExOR	mhBackoff_;	// backoff timer

	/* ============================================================
	   Internal MAC State
	   ============================================================ */
	double		nav_;		// Network Allocation Vector

	MacState	rx_state_;	// incoming state (MAC_RECV or MAC_IDLE)
	MacState	tx_state_;	// outgoint state
	int		tx_active_;	// transmitter is ACTIVE

	Packet          *eotPacket_;    // copy for eot callback

	Packet		*pktRTS_;	// outgoing RTS packet
	Packet		*pktCTRL_;	// outgoing non-RTS packet

	u_int32_t	cw_;		// Contention Window
	u_int32_t	ssrc_;		// STA Short Retry Count
	u_int32_t	slrc_;		// STA Long Retry Count

	int		min_frame_len_;

	NsObject*	logtarget_;
	NsObject*       EOTtarget_;     // given a copy of packet at TX end




	/* ============================================================
	   Duplicate Detection state
	   ============================================================ */
	u_int16_t	sta_seqno_;	// next seqno that I'll use
	int		cache_node_count_;
	Host		*cache_;
};

#endif /* __mac_802_exor__ */

