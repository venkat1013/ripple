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
 * Contributed by Giao Nguyen, http://daedalus.cs.berkeley.edu/~gnguyen
 *
 * @(#) $Header: /tianji.cvsroot/newripple/ns-2.30/mac/mac-csma.h,v 1.1.1.1 2008/12/09 11:15:35 ripple Exp $ (UCB)
 */

#ifndef ns_mac_csma_h
#define ns_mac_csma_h

#include "mac.h"

/*
// Carrier Sense Multiple Access MAC
*/

class MacCsma;

class MacHandlerEoc : public Handler {
public:
	MacHandlerEoc(MacCsma* m) : mac_(m) {}
	void handle(Event* e);
protected:
	MacCsma* mac_;
};


class MacCsma : public Mac {
public:
	MacCsma();
	virtual void send(Packet* p);
	virtual void resume(Packet* p = 0);
	virtual void endofContention(Packet* p);

protected:
	virtual void backoff(Handler* h, Packet* p, double delay=0);
	double txstart_;	// when the transmission starts
	double ifs_;		// interframe spacing
	double slotTime_;	// duration of contention slot in seconds
	int cw_;		// current contention window
	int cwmin_;		// minimum contention window for 1st backoff
	int cwmax_;		// maximum contention window (backoff range)
	int rtx_;		// number of retransmission attempt
	int rtxLimit_;		// maximum number of retransmission attempt
	int csense_;		// carrier sense or not
	Event eEoc_;		// event at the end-of-contention
	MacHandlerEoc hEoc_;	// handle end-of-contention
};


class MacCsmaCd : public MacCsma {
public:
	void endofContention(Packet*);
};


class MacCsmaCa : public MacCsma {
public:
	virtual void send(Packet*);
};

#endif
