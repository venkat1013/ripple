# 
#       Tianji Li
#       Hamilton Institute, NUIM
#       02/07/2008
# 



# ======================================================================
# Define options
# ======================================================================
# ======================================================================
set opt(chan)           Channel/WirelessChannel    ;# channel type
set opt(prop)           Propagation/Shadowing      ;# radio-propagation model
set opt(netif)          Phy/WirelessPhy            ;# network interface type
set opt(mac)            Mac/Ripple                 ;# MAC type
set opt(ifq)            Queue/DropTail/PriQueue    ;# interface queue type
set opt(ll)             LL                         ;# link layer type
set opt(ant)            Antenna/OmniAntenna        ;# antenna model
set opt(ifqlen)         50                         ;# max packet in ifq
set opt(adhocRouting)   NOAH                  ;# routing protocol
set opt(x)              10000                       ;# x coordinate of topology
set opt(y)              10000                       ;# y coordinate of topology
set opt(ShortRetryLimit) 4
set opt(LongRetryLimit)  7
set opt(RTSThreshold)    65536
set opt(CWmin)           15
set opt(CWmax)           1023
set opt(SlotTime)        0.000009
set opt(SIFS)            0.000016
set opt(AIFS)            2
set opt(PktSize)         1000

set opt(rate)            216
set opt(dataRate)        $opt(rate)Mb
set opt(basicRate)       54Mb
set opt(PLCPhdrRate)     $opt(rate)Mb

set opt(stop)           [lindex $argv 0]
set opt(start)          0
set opt(rectime)        0.01
set opt(nn)             5

set opt(fdfnum)         1 ;# number of forward-direction flows
set opt(rdfnum)         0 ;# number of reverse-direction flows


#------------------------------------------------------------------------
# use current time as seed for every run
$defaultRNG seed 0


#   Application 
#------------------------------------------------------------------------
Agent/TCP               set packetSize_   $opt(PktSize)

# PHY
#------------------------------------------------------------------------
Phy/WirelessPhy set CPThresh_   10000.0         ;# used at MAC, this value basically disables capture effect


Phy/WirelessPhy set CSThresh_   2.63769e-11     ;# 30% pkts sensed at 800m
# Phy/WirelessPhy set RXThresh_   2.42494e-12     ;# 90% pkts received at 500m
# Phy/WirelessPhy set RXThresh_   6.75247e-11     ;# 30% pkts received at 500m
# Phy/WirelessPhy set RXThresh_   1.21127e-10     ;# 20% pkts received at 500m
# Phy/WirelessPhy set RXThresh_   2.72385e-10     ;# 10% pkts received at 500m
# Phy/WirelessPhy set RXThresh_   6.42514e-10     ;# 50% pkts received at 100m
# Phy/WirelessPhy set RXThresh_   6.80963e-09     ;# 10% pkts received at 100m
# Phy/WirelessPhy set RXThresh_   3.02817e-09     ;# 20% pkts received at 100m
Phy/WirelessPhy set RXThresh_   2.44547e-10     ;# 70% pkts received at 100m
Phy/WirelessPhy set Pt_         0.28183815      ;# watts

# MAC
#------------------------------------------------------------------------
Mac/Ripple set BER_               0.0000  ;# not useful, shadowing model is good enough
Mac/Ripple set basicRate_         $opt(basicRate)
Mac/Ripple set dataRate_          $opt(dataRate)
Mac/Ripple set CWMin_             $opt(CWmin)
Mac/Ripple set CWMax_             $opt(CWmax)
Mac/Ripple set SlotTime_          $opt(SlotTime)
Mac/Ripple set SIFS_              $opt(SIFS)
Mac/Ripple set ShortRetryLimit_   $opt(ShortRetryLimit)
Mac/Ripple set LongRetryLimit_    $opt(LongRetryLimit)
Mac/Ripple set RTSThreshold_      $opt(RTSThreshold)
Mac/Ripple set MAX_Sq_SIZE_       $opt(ifqlen)
Mac/Ripple set num_sta_           $opt(nn)
LL set totNumSTA_                 [expr $opt(nn)-1]
# LL set delay_                     0us

# create simulator instance
#------------------------------------------------------------------------
remove-packet-header AODV ARP IMEP IPinIP IVS LDP MPLS MIP Ping PGM PGM_SPM PGM_NAK NV Smac Pushback TORA TFRC_ACK TFRC 
set ns_   [new Simulator]
# $ns_ at $opt(rectime) "print_stuff"
set tracefd  [open aaaa.tr w]
set namtrace [open aaaa.nam w]
$ns_ trace-all $tracefd
$ns_ namtrace-all-wireless $namtrace $opt(x) $opt(y)
set topo   [new Topography]
$topo load_flatgrid $opt(x) $opt(y)

set opt(nnhtp)          2
set opt(machtp)         Mac/802_11                 ;# MAC type
create-god [expr $opt(nn)+$opt(nnhtp)]

# configure for base-station node
$ns_ node-config -adhocRouting $opt(adhocRouting) \
                 -llType $opt(ll) \
                 -macType $opt(mac) \
                 -ifqType $opt(ifq) \
                 -ifqLen $opt(ifqlen) \
                 -antType $opt(ant) \
                 -propType $opt(prop) \
                 -phyType $opt(netif) \
                 -channelType $opt(chan) \
                 -topoInstance $topo \
                 -agentTrace ON \
                 -ifqTrace ON \
                 -routerTrace ON \
                 -macTrace ON 

#create stations
#------------------------------------------------------------------------
for {set i 0} {$i < $opt(nn)} {incr i} {
    if { $i == 0 } {
        LL set fdMacAddr_ [expr $opt(nn)-1]
        LL set rdMacAddr_ -1
    } elseif { $i == ($opt(nn)-1) } {
        LL set fdMacAddr_ -1
        LL set rdMacAddr_ 0
    } else {
        LL set fdMacAddr_ [expr $opt(nn)-1]
        LL set rdMacAddr_ 0
    }
    set node_($i) [$ns_ node]   

    if { $i == 0 } {
        $node_($i) set X_ 2891
        $node_($i) set Y_ 2243
    } 
    if { $i == 1 } {
        $node_($i) set X_ 2739
        $node_($i) set Y_ 1843
    } 
    if { $i == 2 } {
        $node_($i) set X_ 2572
        $node_($i) set Y_ 1512
    } 
    if { $i == 3 } {
        $node_($i) set X_ 2487
        $node_($i) set Y_ 1129
    } 
    if { $i == 4 } {
        $node_($i) set X_ 2190
        $node_($i) set Y_ 1116
    } 


    $node_($i) set Z_ 0.0
    $node_($i) random-motion 0          ;# disable random motion
}


# forward-direction flows
set fdflowid 0
for {set i 0} {$i < $opt(fdfnum)} {incr i 1} {
    set fdtcp($fdflowid) [new Agent/TCP]
    set fdsink($fdflowid) [new Agent/TCPSink]
    $ns_ attach-agent $node_(0) $fdtcp($fdflowid)
    $ns_ attach-agent $node_([expr $opt(nn)-1]) $fdsink($fdflowid)
    $ns_ connect $fdtcp($fdflowid) $fdsink($fdflowid)

    set fdapp($fdflowid) [new Application/FTP]
    $fdapp($fdflowid) attach-agent $fdtcp($fdflowid)

    if { $i == 0 } {
        $fdtcp($fdflowid)  ForwardList 0 1 2 3 4 
        $fdsink($fdflowid) ForwardList 4 3 2 1 0
    }
    $fdtcp($fdflowid)  MacType 3 ;#   AFR=0, LPR=1, SPR=2, ETX=2, Ripple=3
    $fdsink($fdflowid) MacType 3
    $ns_ at $opt(start) "$fdapp($fdflowid) start" 
    set fdflowid [expr $fdflowid + 1]
}

# reverse-direction flows
set rdflowid 0
for {set i 0} {$i < $opt(rdfnum)} {incr i 1} {
    set rdtcp($rdflowid) [new Agent/TCP]
    set rdsink($rdflowid) [new Agent/TCPSink]
    $ns_ attach-agent $node_([expr $opt(nn)-1]) $rdtcp($rdflowid)
    $ns_ attach-agent $node_(0) $rdsink($rdflowid)
    $ns_ connect $rdtcp($rdflowid) $rdsink($rdflowid)

    set rdapp($rdflowid) [new Application/FTP]
    $rdapp($rdflowid) attach-agent $rdtcp($rdflowid)

#     set f_cwnd  [open "out.rd.cwnd.flow.$rdflowid.txt" "w"]
#     set f_throughput  [open "out.rd.thru.flow.$rdflowid.txt" "w"]
#     set f_rtt  [open "out.rd.rtt.flow.$rdflowid.txt" "w"]
#     close $f_cwnd 
#     close $f_throughput 
#     close $f_rtt 

    $ns_ at $opt(start) "$rdapp($rdflowid) start" 
    set rdflowid [expr $rdflowid + 1]
}




############################################################
############################################################
#
#    HTPs from 4->5
#
############################################################
############################################################
# MAC
#------------------------------------------------------------------------
Mac/802_11 set BER_               0.0000  ;# not useful, shadowing model is good enough
Mac/802_11 set basicRate_         $opt(basicRate)
Mac/802_11 set dataRate_          $opt(dataRate)
Mac/802_11 set CWMin_             $opt(CWmin)
Mac/802_11 set CWMax_             $opt(CWmax)
Mac/802_11 set SlotTime_          $opt(SlotTime)
Mac/802_11 set SIFS_              $opt(SIFS)
Mac/802_11 set ShortRetryLimit_   $opt(ShortRetryLimit)
Mac/802_11 set LongRetryLimit_    $opt(LongRetryLimit)
Mac/802_11 set RTSThreshold_      $opt(RTSThreshold)
Mac/802_11 set MAX_Sq_SIZE_       $opt(ifqlen)
Mac/802_11 set nodes              [expr $opt(nn)+1]

# configure for base-station node
$ns_ node-config -adhocRouting $opt(adhocRouting) \
                 -llType $opt(ll) \
                 -macType $opt(machtp) \
                 -ifqType $opt(ifq) \
                 -ifqLen $opt(ifqlen) \
                 -antType $opt(ant) \
                 -propType $opt(prop) \
                 -phyType $opt(netif) \
                 -channelType $opt(chan) \
                 -topoInstance $topo \
                 -agentTrace ON \
                 -ifqTrace ON \
                 -routerTrace ON \
                 -macTrace ON 
#create stations
#------------------------------------------------------------------------
for {set i $opt(nn)} {$i < $opt(nn)+$opt(nnhtp)} {incr i} {
    if { $i == $opt(nn) } {
        LL set fdMacAddr_ [expr $i+1]
        LL set rdMacAddr_ -1
    } 

    if { $i == ($opt(nn)+$opt(nnhtp)-1) } {
        LL set fdMacAddr_ $opt(nn)
        LL set rdMacAddr_ $opt(nn)
    } 

    set node_($i) [$ns_ node]   

    if { $i == $opt(nn) } {
        $node_($i) set X_ 2664
        $node_($i) set Y_ 1204
    }
    if { $i == ($opt(nn)+$opt(nnhtp)-1) } {
        $node_($i) set X_ 3158
        $node_($i) set Y_ 871
    }



    $node_($i) set Z_ 0.0
    $node_($i) random-motion 0          ;# disable random motion
}

# forward-direction flows
set htpflowid 0
for {set i 0} {$i < 1} {incr i 1} {
    set htptcp($htpflowid) [new Agent/TCP]
    set htpsink($htpflowid) [new Agent/TCPSink]
    $ns_ attach-agent $node_($opt(nn)) $htptcp($htpflowid)
    $ns_ attach-agent $node_([expr $opt(nn)+$opt(nnhtp)-1]) $htpsink($htpflowid)
    $ns_ connect $htptcp($htpflowid) $htpsink($htpflowid)

    set htpapp($htpflowid) [new Application/FTP]
    $htpapp($htpflowid) attach-agent $htptcp($htpflowid)

    set f_cwnd  [open "out.htp.cwnd.flow.$htpflowid.txt" "w"]
    set f_throughput  [open "out.htp.thru.flow.$htpflowid.txt" "w"]
    set f_rtt  [open "out.htp.rtt.flow.$htpflowid.txt" "w"]
    close $f_cwnd 
    close $f_throughput 
    close $f_rtt 

    $ns_ at $opt(start) "$htpapp($htpflowid) send 5000000" 
    set htpflowid [expr $htpflowid + 1]
}









# Result output routines
#########################################################################
$ns_ at $opt(stop) "stop"
$ns_ at $opt(stop) "$ns_ halt"

proc stop {} {
    global ns_ tracefd namtrace node_ opt  fdtcp fdsink rdtcp rdsink htptcp htpsink
        set now [$ns_ now]

        set fdflowid 0
        set fdsum 0.0
        set f_throughput  [open "results.exor.topo.txt" "a"]
        puts $f_throughput "================================================="
        set aaa "+DATE(month/day/year): %m/%d/%Y, TIME: %H:%M:%S"
        set systemtime       "[exec date $aaa]"
        puts $f_throughput "$systemtime"
        puts $f_throughput "rate: $opt(rate) Mbps"
        puts $f_throughput "run: $opt(stop) seconds"
        puts $f_throughput "================================================="
        puts -nonewline  $f_throughput "fdflows: "
        for {set i 0} {$i < $opt(fdfnum)} {incr i} {
                set t_thru   [string range  [expr [$fdsink($fdflowid) set bytes_].0*8/1000000/$now] 0 6]
                set fdsum [expr $fdsum+$t_thru]
                puts -nonewline  $f_throughput  " $t_thru "
                set fdflowid [expr $fdflowid + 1]
        }
        puts $f_throughput "  sum: $fdsum"


        puts -nonewline  $f_throughput "rdflows: "
        set rdflowid 0
        set rdsum 0.0
        for {set i 0} {$i < $opt(rdfnum)} {incr i} {
                set t_thru   [string range  [expr [$rdsink($rdflowid) set bytes_].0*8/1000000/$now] 0 6]
                set rdsum [expr $rdsum+$t_thru]
                puts  -nonewline $f_throughput  " $t_thru "
                set rdflowid [expr $rdflowid + 1]
        }
        puts $f_throughput "  sum: $rdsum"
#         puts $f_throughput "================================================="
#         puts $f_throughput "      "
#         puts $f_throughput "      "

        puts -nonewline  $f_throughput "htpflows: "
        set htpflowid 0
        set htpsum 0.0
        for {set i 0} {$i < 1} {incr i} {
                set t_thru   [string range  [expr [$htpsink($htpflowid) set bytes_].0*8/1000000/$now] 0 6]
                set htpsum [expr $htpsum+$t_thru]
                puts  -nonewline $f_throughput  " $t_thru "
                set htpflowid [expr $htpflowid + 1]
        }
        puts $f_throughput "  sum: $htpsum"
        puts $f_throughput "================================================="
        puts $f_throughput "      "
        puts $f_throughput "      "
        close $f_throughput 

    exec rm -rf aaaa.nam aaaa.tr
}



set step 0.01
proc print_stuff { } {
        global ns_ opt step fdtcp fdsink rdtcp rdsink
        set now [$ns_ now]

        set t_now  [string range $now 0 5]

        set fdflowid 0
        for {set i 0} {$i < $opt(fdfnum)} {incr i} {
                set t_cwnd   [string range [$fdtcp($fdflowid) set cwnd_] 0 4]
                set t_thru   [string range  [expr [$fdsink($fdflowid) set bytes_].0*8/1000000/$now] 0 6]
                set t_rtt    [string range  [$fdtcp($fdflowid) set srtt_] 0 4]

                set f_cwnd  [open "out.fd.cwnd.flow.$fdflowid.txt" "a"]
                set f_throughput  [open "out.fd.thru.flow.$fdflowid.txt" "a"]
                set f_rtt  [open "out.fd.rtt.flow.$fdflowid.txt" "a"]

                puts $f_cwnd  "$t_now $t_cwnd "
                puts $f_throughput  "$t_now $t_thru "
                puts $f_rtt  "$t_now $t_rtt "

                close $f_cwnd 
                close $f_throughput 
                close $f_rtt

                set fdflowid [expr $fdflowid + 1]
        }


        set rdflowid 0
        for {set i 0} {$i < $opt(rdfnum)} {incr i} {
                set t_cwnd   [string range [$rdtcp($rdflowid) set cwnd_] 0 4]
                set t_thru   [string range  [expr [$rdsink($rdflowid) set bytes_].0*8/1000000/$now] 0 6]
                set t_rtt    [string range [$rdtcp($rdflowid) set srtt_] 0 4]

                set f_rtt   [open "out.rd.cwnd.flow.$rdflowid.txt" "a"]
                set f_cwnd  [open "out.rd.cwnd.flow.$rdflowid.txt" "a"]
                set f_throughput  [open "out.rd.thru.flow.$rdflowid.txt" "a"]

                puts $f_rtt  "$t_now $t_rtt "
                puts $f_cwnd  "$t_now $t_cwnd "
                puts $f_throughput  "$t_now $t_thru "

                close $f_rtt 
                close $f_cwnd 
                close $f_throughput 
                set rdflowid [expr $rdflowid + 1]
        }

        $ns_ at [expr $now + $step] "print_stuff"
}




puts "Starting Simulation..."
$ns_ run

