#
# Copyright (c) 1997 Regents of the University of California.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this software
#    must display the following acknowledgement:
# 	This product includes software developed by the Daedalus Research
#       Group at the University of California, Berkeley.
# 4. Neither the name of the University nor of the research group
#    may be used to endorse or promote products derived from this software 
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# Contributed by the Daedalus Research Group, http://daedalus.cs.berkeley.edu
#

#default channel propagation delay (for a LAN)
Channel set delay_ 4us

Classifier/Mac set bcast_ 0

#default bandwidth setting done during mac initialisation (c++)
Mac set bandwidth_ 2Mb
Mac set delay_ 0us

Mac/Simple set fullduplex_mode_ 0

# IEEE 802.11 MAC settings
if [TclObject is-class Mac/802_11] {
        Mac/802_11 set delay_ 64us
        Mac/802_11 set ifs_ 16us
        Mac/802_11 set slotTime_ 16us
        Mac/802_11 set cwmin_ 16
        Mac/802_11 set cwmax_ 1024
        Mac/802_11 set rtxLimit_ 16
        Mac/802_11 set bssId_ -1
        Mac/802_11 set sifs_ 8us
        Mac/802_11 set pifs_ 12us
        Mac/802_11 set difs_ 16us
        Mac/802_11 set rtxAckLimit_ 1
        Mac/802_11 set rtxRtsLimit_ 3
        Mac/802_11 set basicRate_ 1Mb  ;# set this to 0 if want to use bandwidth_ for 
        Mac/802_11 set dataRate_ 1Mb   ;# both control and data pkts
}

# Tianji Li, Jul. 06, 2008
if [TclObject is-class Mac/Ripple] {
        Mac/Ripple set delay_ 64us
        Mac/Ripple set ifs_ 16us
        Mac/Ripple set slotTime_ 16us
        Mac/Ripple set cwmin_ 16
        Mac/Ripple set cwmax_ 1024
        Mac/Ripple set rtxLimit_ 16
        Mac/Ripple set bssId_ -1
        Mac/Ripple set sifs_ 8us
        Mac/Ripple set pifs_ 12us
        Mac/Ripple set difs_ 16us
        Mac/Ripple set rtxAckLimit_ 1
        Mac/Ripple set rtxRtsLimit_ 3
        Mac/Ripple set basicRate_ 1Mb  ;# set this to 0 if want to use bandwidth_ for 
        Mac/Ripple set dataRate_ 1Mb   ;# both control and data pkts
}

if [TclObject is-class Mac/One] {
        Mac/One set delay_ 64us
        Mac/One set ifs_ 16us
        Mac/One set slotTime_ 16us
        Mac/One set cwmin_ 16
        Mac/One set cwmax_ 1024
        Mac/One set rtxLimit_ 16
        Mac/One set bssId_ -1
        Mac/One set sifs_ 8us
        Mac/One set pifs_ 12us
        Mac/One set difs_ 16us
        Mac/One set rtxAckLimit_ 1
        Mac/One set rtxRtsLimit_ 3
        Mac/One set basicRate_ 1Mb  ;# set this to 0 if want to use bandwidth_ for 
        Mac/One set dataRate_ 1Mb   ;# both control and data pkts
}

if [TclObject is-class Mac/ExOR] {
        Mac/ExOR set delay_ 64us
        Mac/ExOR set ifs_ 16us
        Mac/ExOR set slotTime_ 16us
        Mac/ExOR set cwmin_ 16
        Mac/ExOR set cwmax_ 1024
        Mac/ExOR set rtxLimit_ 16
        Mac/ExOR set bssId_ -1
        Mac/ExOR set sifs_ 8us
        Mac/ExOR set pifs_ 12us
        Mac/ExOR set difs_ 16us
        Mac/ExOR set rtxAckLimit_ 1
        Mac/ExOR set rtxRtsLimit_ 3
        Mac/ExOR set basicRate_ 1Mb  ;# set this to 0 if want to use bandwidth_ for 
        Mac/ExOR set dataRate_ 1Mb   ;# both control and data pkts
}
if [TclObject is-class Mac/MCExOR] {
        Mac/MCExOR set delay_ 64us
        Mac/MCExOR set ifs_ 16us
        Mac/MCExOR set slotTime_ 16us
        Mac/MCExOR set cwmin_ 16
        Mac/MCExOR set cwmax_ 1024
        Mac/MCExOR set rtxLimit_ 16
        Mac/MCExOR set bssId_ -1
        Mac/MCExOR set sifs_ 8us
        Mac/MCExOR set pifs_ 12us
        Mac/MCExOR set difs_ 16us
        Mac/MCExOR set rtxAckLimit_ 1
        Mac/MCExOR set rtxRtsLimit_ 3
        Mac/MCExOR set basicRate_ 1Mb  ;# set this to 0 if want to use bandwidth_ for 
        Mac/MCExOR set dataRate_ 1Mb   ;# both control and data pkts
}


# IEEE 802.11e MAC settings
# if [TclObject is-class Mac/802_11e] {
#         Mac/802_11e set delay_ 64us
#         Mac/802_11e set ifs_ 16us
#         Mac/802_11e set slotTime_ 16us
#         Mac/802_11e set cwmin_ 16
#         Mac/802_11e set cwmax_ 1024
#         Mac/802_11e set rtxLimit_ 16
#         Mac/802_11e set bssId_ -1
#         Mac/802_11e set sifs_ 8us
#         Mac/802_11e set pifs_ 12us
#         Mac/802_11e set difs_ 16us
#         Mac/802_11e set rtxAckLimit_ 1
#         Mac/802_11e set rtxRtsLimit_ 3
#         Mac/802_11e set basicRate_ 1Mb  ;# set this to 0 if want to use bandwidth_ for 
#         Mac/802_11e set dataRate_ 1Mb   ;# both control and data pkts
#         Mac/802_11e set cfb_ 0 ;# disables CFB
# }

# IEEE 802.14 MAC settings
if [TclObject is-class Mac/Mcns] {
	Mac/Mcns set bandwidth_ 10Mb
	Mac/Mcns set hlen_ 6
	Mac/Mcns set bssId_ -1
	Mac/Mcns set slotTime_ 10us
}

# Multihop wireless MAC modeled after Metricom's Ricochet
if [TclObject is-class Mac/Multihop] {
	Mac/Multihop set bandwidth_ 100Kb
	Mac/Multihop set delay_ 10ms
	Mac/Multihop set tx_rx_ 11.125ms
	Mac/Multihop set rx_tx_ 13.25ms
	Mac/Multihop set rx_rx_ 10.5625
	Mac/Multihop set backoffBase_ 20ms
	Mac/Multihop set hlen_ 16
}

# The MAC classifier (to demux incoming packets to the correct LL object)
Mac instproc classify-macs {peerinfo} {
	set peerlabel [lindex $peerinfo 0]
	set peerll [lindex $peerinfo 1]
	$self instvar mclass_
	set mclass_ [new Classifier/Mac]
	$mclass_ install $peerlabel $peerll
	$self target $mclass_
}

# XXX this belongs in ns-node.tcl
# Hook up the MACs together at a given Node
Node instproc addmac {mac} { 
	$self instvar machead_ mactail_

	if ![info exists mactail_] {
		set mactail_ [set machead_ $mac]
		$mac maclist $mactail_
	} else {
		$mactail_ maclist $mac
		$mac maclist $machead_
		set mactail_ $mac
	}
}
