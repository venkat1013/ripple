/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 1994-1997 Regents of the University of California.
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
 */

#ifndef lint
static const char rcsid[] =
    "@(#) $Header: /tianji.cvsroot/newripple/ns-2.30/common/packet.cc,v 1.6 2009/04/10 10:40:38 ripple Exp $ (LBL)";
#endif

#include "packet.h"
#include "flags.h"
#include "mac.h"
#include "mac-ripple.h"
#include "mac/exor/mac-exor.h"
#include "mac/mcexor/mac-mcexor.h"
#include "object.h"

p_info packet_info;
// char* p_info::name_[PT_NTYPE+1];

int Packet::hdrlen_ = 0;		// size of a packet's header
Packet* Packet::free_;			// free list
int hdr_cmn::offset_;			// static offset of common header
int hdr_flags::offset_;			// static offset of flags header


PacketHeaderClass::PacketHeaderClass(const char* classname, int hdrlen) :
	TclClass(classname), hdrlen_(hdrlen), offset_(0)
{
}


TclObject* PacketHeaderClass::create(int, const char*const*)
{
	return (0);
}

void PacketHeaderClass::bind()
{
	TclClass::bind();
	Tcl& tcl = Tcl::instance();
	tcl.evalf("%s set hdrlen_ %d", classname_, hdrlen_);
// printf("%s set hdrlen_ %d\n", classname_, hdrlen_);
	export_offsets();
	add_method("offset");
}

void PacketHeaderClass::export_offsets()
{
}

void PacketHeaderClass::field_offset(const char* fieldname, int offset)
{
	Tcl& tcl = Tcl::instance();
	tcl.evalf("%s set offset_(%s) %d", classname_, fieldname, offset);
// printf("%s set offset_(%s) %d\n", classname_, fieldname, offset);
}


int PacketHeaderClass::method(int ac, const char*const* av)
{
	Tcl& tcl = Tcl::instance();
	int argc = ac - 2;
	const char*const* argv = av + 2;
	if (argc == 3) {
		if (strcmp(argv[1], "offset") == 0) {
			if (offset_) {
				*offset_ = atoi(argv[2]);
//#ifdef TIANJI_RIPPLE
//printf("PacketHeaderClass::method, offset=%d\n", atoi(argv[2]));
//#endif
				return TCL_OK;
			}
			tcl.resultf("Warning: cannot set offset_ for %s",
				    classname_);
			return TCL_OK;
		}
	}
	else if (argc == 2) {
		if (strcmp(argv[1], "offset") == 0) {
			if (offset_) {
				tcl.resultf("%d", *offset_);
				return TCL_OK;
			}
		}
	}
	return TclClass::method(ac, av);
}


class CommonHeaderClass : public PacketHeaderClass {
public:
	CommonHeaderClass() : PacketHeaderClass("PacketHeader/Common",
						sizeof(hdr_cmn)) {
		bind_offset(&hdr_cmn::offset_);
	}
	void export_offsets() {
		field_offset("ptype_", OFFSET(hdr_cmn, ptype_));
		field_offset("size_", OFFSET(hdr_cmn, size_));
		field_offset("uid_", OFFSET(hdr_cmn, uid_));
		field_offset("error_", OFFSET(hdr_cmn, error_));
	};
} class_cmnhdr;

class FlagsHeaderClass : public PacketHeaderClass {
public:
	FlagsHeaderClass() : PacketHeaderClass("PacketHeader/Flags",
						sizeof(hdr_flags)) {
		bind_offset(&hdr_flags::offset_);
	}
} class_flagshdr;


/* manages active packet header types */
class PacketHeaderManager : public TclObject {
public:
	PacketHeaderManager() {
		bind("hdrlen_", &Packet::hdrlen_);
// printf("PacketHeaderManager init, hdrlen=%d\n",Packet::hdrlen_);
	}
};

static class PacketHeaderManagerClass : public TclClass {
public:
	PacketHeaderManagerClass() : TclClass("PacketHeaderManager") {}
	TclObject* create(int, const char*const*) {
		return (new PacketHeaderManager);
	}
} class_packethdr_mgr;




#ifdef TIANJI_RIPPLE
unsigned char* PacketData::access_data(int off)
{
		if (off < 0) abort();
		return (&data_[off]);
}


void Packet::free(Packet* p)
{
	struct hdr_cmn    * ch;
	PacketData        * l_data;
	struct data_part  * l_data_p;

	if (p->fflag()) {
		if (p->ref_count() == 0) {
			assert(p->uid_ <= 0);

			if (p->data_ != 0) {

				// free NS pkts for RIPPLE data frames
				ch = HDR_CMN(p);
				if ( ch->ptype() == PT_MAC_RIPPLE_DATA || ch->ptype() == PT_MAC_EXOR_DATA || ch->ptype() == PT_MAC_MCEXOR_DATA ) {
					l_data = (PacketData *)p->userdata();
					for ( int i=0; i<ch->num_pkts(); i++) {
						l_data_p  = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
						if ( l_data_p->NSpkt != NULL)
//							printf("Packet::free, does this ever happen??\n");
							Packet::free(l_data_p->NSpkt);
							l_data_p->NSpkt = 0;
					}
				}

				delete p->data_;
				p->data_ = 0;
			}
			init(p);
			p->next_ = free_;
			free_ = p;
			p->fflag_ = FALSE;
		} else {
			--p->ref_count();
		}
	}
}

// do not free NS pkts in RIPPLE data frames
void Packet::freehdrs(Packet* p)
{
	if (p->fflag()) {
		if (p->ref_count() == 0) {
			assert(p->uid_ <= 0);

			if (p->data_ != 0) {
				delete p->data_;
				p->data_ = 0;
			}
			init(p);
			p->next_ = free_;
			free_ = p;
			p->fflag_ = FALSE;
		} else {
			--p->ref_count();
		}
	}
}


Packet*
Packet::copy()
{
	struct hdr_cmn    * ch;
	PacketData        * l_data;
	struct data_part  * l_data_p;
//printf("Packet::copy, before\n");
//printf(UID_PRINTF_FORMAT, this->uid_ );
//printf("\n");

	Packet* p = alloc();
	memcpy(p->bits(), bits(), hdrlen());
	if (data_){
		p->data_ = data_->copy();
		ch = HDR_CMN(p);

		// copy NS pkts too for RIPPLE data frames, otherwise, this function copys pointers only
		if ( ch->ptype() == PT_MAC_RIPPLE_DATA || ch->ptype() == PT_MAC_EXOR_DATA  || ch->ptype() == PT_MAC_MCEXOR_DATA) {
			l_data = (PacketData *)p->userdata();
				for ( int i=0; i<ch->num_pkts(); i++) {
					l_data_p         = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
					l_data_p->NSpkt  = l_data_p->NSpkt->copy();
				}
		}

	}
	p->txinfo_.init(&txinfo_);

//	printf("Packet::copy, after\n");
//	printf(UID_PRINTF_FORMAT, p->uid_ );
//	printf("\n");

	return (p);
}


Packet* Packet::alloc()
{
	Packet* p = free_;

	if (p != 0) {
		assert(p->fflag_ == FALSE);
		free_ = p->next_;
		assert(p->data_ == 0);
		p->uid_ = 0;
		p->time_ = 0;
	} else {
		p = new Packet;
		p->bits_ = new unsigned char[hdrlen_];
		if (p == 0 || p->bits_ == 0)
			abort();
	}

	init(p); // Initialize bits_[]
	(HDR_CMN(p))->next_hop_ = -2; // -1 reserved for IP_BROADCAST
	(HDR_CMN(p))->last_hop_ = -2; // -1 reserved for IP_BROADCAST
	p->fflag_ = TRUE;
	(HDR_CMN(p))->direction() = hdr_cmn::DOWN;
	p->next_ = 0;
	return (p);
}


#endif






//#ifdef TIANJI_RIPPLE
//unsigned char* PacketData::access_data(int off)
//{
//		if (off < 0) abort();
//		return (&data_[off]);
//}
//
//
//void Packet::free(Packet* p)
//{
//	struct hdr_cmn    * ch;
//	PacketData        * l_data;
//	struct data_part  * l_data_p;
//
//	if (p->fflag()) {
//		if (p->ref_count() == 0) {
//			assert(p->uid_ <= 0);
//
//			if (p->data_ != 0) {
//
//				// free NS pkts for RIPPLE data frames
//				ch = HDR_CMN(p);
//				if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
//					l_data = (PacketData *)p->userdata();
//					for ( int i=0; i<ch->num_pkts(); i++) {
//						l_data_p  = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
//						if ( l_data_p->NSpkt != NULL)
//							printf("Packet::free, does this ever happen??\n");
//							Packet::free(l_data_p->NSpkt);
//							l_data_p->NSpkt = 0;
//					}
//				}
//
//				delete p->data_;
//				p->data_ = 0;
//			}
//
//			delete p;
//			p = 0;
//		} else {
//			--p->ref_count();
//		}
//	}
//}
//
//
//// do not free NS pkts in RIPPLE data frames
//void Packet::freehdrs(Packet* p)
//{
//	if (p->fflag()) {
//		if (p->ref_count() == 0) {
//			assert(p->uid_ <= 0);
//
//			if (p->data_ != 0) {
//				delete p->data_;
//				p->data_ = 0;
//			}
//
//			delete p;
//			p = 0;
//		} else {
//			--p->ref_count();
//		}
//	}
//}
//
//
//Packet*
//Packet::copy()
//{
//	struct hdr_cmn    * ch;
//	PacketData        * l_data;
//	struct data_part  * l_data_p;
//
//	Packet* p = alloc();
//	memcpy(p->bits(), bits(), hdrlen());
//	if (data_){
//		p->data_ = data_->copy();
//		ch = HDR_CMN(p);
//
//		// copy NS pkts too for RIPPLE data frames, otherwise, this function copys pointers only
//		if ( ch->ptype() == PT_MAC_RIPPLE_DATA ) {
//			l_data = (PacketData *)p->userdata();
//				for ( int i=0; i<ch->num_pkts(); i++) {
//						l_data_p         = (struct data_part *)l_data->access_data( i * sizeof(struct data_part) );
//						l_data_p->NSpkt  = l_data_p->NSpkt->copy();
//				}
//		}
//
//	}
//	p->txinfo_.init(&txinfo_);
//
//	return (p);
//}
//
//
//Packet* Packet::alloc()
//{
//	Packet* p;
//
//	p = new Packet;
//	p->bits_ = new unsigned char[hdrlen_];
//	if (p == 0 || p->bits_ == 0)
//		abort();
//
//	init(p); // Initialize bits_[]
//	(HDR_CMN(p))->next_hop_ = -2; // -1 reserved for IP_BROADCAST
//	(HDR_CMN(p))->last_hop_ = -2; // -1 reserved for IP_BROADCAST
//	p->fflag_ = TRUE;
//	(HDR_CMN(p))->direction() = hdr_cmn::DOWN;
//	p->next_ = 0;
//	return (p);
//}
//
//
//#endif
