/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*-
 *
 * Copyright (c) 1994 Regents of the University of California.
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
    "@(#) $Header: /tianji.cvsroot/newripple/ns-2.30/common/object.cc,v 1.3 2009/07/06 14:50:01 ripple Exp $ (LBL)";
#endif

#include <stdarg.h>

#include "object.h"
#include "packet.h"
#include "flags.h"

//#define TIANJI_NEW_DEBUG

NsObject::~NsObject()
{
}

NsObject::NsObject()
{
	// Turn off debug by default
	debug_ = 0;
}

void
NsObject::delay_bind_init_all()
{
	delay_bind_init_one("debug_");
}

int
NsObject::delay_bind_dispatch(const char *varName, const char *localName, TclObject *tracer)
{
	if (delay_bind_bool(varName, localName, "debug_", &debug_, tracer))
		return TCL_OK;
	return TclObject::delay_bind_dispatch(varName, localName, tracer);
}

void NsObject::reset()
{
}

int NsObject::command(int argc, const char*const* argv)
{
	if (argc == 2) {
		if (strcmp(argv[1], "reset") == 0) {
			reset();
			return (TCL_OK);
		}
	}
	return (TclObject::command(argc, argv));
}

/*
 * Packets may be handed to NsObjects at sheduled points
 * in time since a node is an event handler and a packet
 * is an event.  Packets should be the only type of event
 * scheduled on a node so we can carry out the cast below.
 */
void NsObject::handle(Event* e)
{
#ifdef TIANJI_NEW_DEBUG
    hdr_cmn *hdr = HDR_CMN((Packet *)e);
	printf("NsObject::handle, event id=%d, uid=%d, forwarders=%d %d %d %d %d %d %d %d, mac_type=%d. \n",
			e->uid_, hdr->uid(), hdr->f_list[0], hdr->f_list[1], hdr->f_list[2],
			hdr->f_list[3], hdr->f_list[4], hdr->f_list[5], hdr->f_list[6], hdr->f_list[7], hdr->mac_type);
	if ( hdr->mac_type == 0	&& hdr->f_list[2] == 0)
		printf("aaaaaaaaa!!!!!!!!\n");
#endif
	recv((Packet*)e);
}

void NsObject::recv(Packet *p, const char*)
{

	Packet::free(p);
}

// Debugging output for all TclObjects. By default, print to stdout
void NsObject::debug(const char *fmt, ...)
{
	if (!debug_)
		return;
	va_list ap;
	va_start(ap, fmt);
	vprintf(fmt, ap);
}
