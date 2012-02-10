/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) Xerox Corporation 1997. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Linking this file statically or dynamically with other modules is making
 * a combined work based on this file.  Thus, the terms and conditions of
 * the GNU General Public License cover the whole combination.
 *
 * In addition, as a special exception, the copyright holders of this file
 * give you permission to combine this file with free software programs or
 * libraries that are released under the GNU LGPL and with code included in
 * the standard release of ns-2 under the Apache 2.0 license or under
 * otherwise-compatible licenses with advertising requirements (or modified
 * versions of such code, with unchanged license).  You may copy and
 * distribute such a system following the terms of the GNU GPL for this
 * file and the licenses of the other code concerned, provided that you
 * include the source code of that other code when and as the GNU GPL
 * requires distribution of source code.
 *
 * Note that people who make modified versions of this file are not
 * obligated to grant this special exception for their modified versions;
 * it is their choice whether to do so.  The GNU General Public License
 * gives permission to release a modified version without this exception;
 * this exception also makes it possible to release a modified version
 * which carries forward this exception.
 */
#ifndef lint
static const char rcsid[] =
	"@(#) $Header: /tianji.cvsroot/newripple/ns-2.30/adc/expavg-est.cc,v 1.1.1.1 2008/12/09 11:15:34 ripple Exp $";
#endif


#include <math.h>
#include "estimator.h"

class ExpAvg_Est : public Estimator {
public:
	ExpAvg_Est() {bind("w_",&w_);};
protected:
	void estimate();
	double w_;
};


void ExpAvg_Est::estimate()
{
	avload_=(1-w_)*avload_+w_*meas_mod_->bitcnt()/period_;
	//printf("%f %f %f\n",Scheduler::instance().clock(),avload_,meas_mod_->bitcnt()/period_);
	fflush(stdout);
	meas_mod_->resetbitcnt(); 
}

static class ExpAvg_EstClass : public TclClass {
public:
	ExpAvg_EstClass() : TclClass ("Est/ExpAvg") {}
	TclObject* create(int,const char*const*) {
		return (new ExpAvg_Est());
	}
}class_expavg_est;
