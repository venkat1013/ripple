# 
# 	Tianji Li
# 	Hamilton Institute, NUIM
# 	02/07/2008
# 



# ======================================================================
# Define options
# ======================================================================
# ======================================================================
set opt(chan)           Channel/WirelessChannel    ;# channel type
set opt(prop)           Propagation/Shadowing      ;# radio-propagation model
set opt(netif)          Phy/WirelessPhy            ;# network interface type
set opt(mac)            Mac/802_11                 ;# MAC type
set opt(ifq)            Queue/DropTail/PriQueue    ;# interface queue type
set opt(ll)             LL                         ;# link layer type
set opt(ant)            Antenna/OmniAntenna        ;# antenna model
set opt(ifqlen)         50                         ;# max packet in ifq
set opt(adhocRouting)   NOAH                       ;# routing protocol
set opt(x)              1000                       ;# x coordinate of topology
set opt(y)              1000                       ;# y coordinate of topology
set opt(ShortRetryLimit) 11
set opt(LongRetryLimit)  11
set opt(RTSThreshold)    65536
set opt(CWmin)           15
set opt(CWmax)           1023
set opt(SlotTime)        0.000009
set opt(SIFS)            0.000016
set opt(AIFS)            2
set opt(PktSize)         1000

set opt(rate)            [lindex $argv 1]
set opt(dataRate)        $opt(rate)Mb
set opt(basicRate)       [lindex $argv 2]Mb
set opt(PLCPhdrRate)     $opt(rate)Mb

set opt(stop)           [lindex $argv 0]
set opt(start)          0
set opt(rectime)        0.01


set opt(nn)             4
set opt(fnum)           [lindex $argv 3]
set opt(BER)            [lindex $argv 4]
Mac/Ripple set BER_     $opt(BER)

#------------------------------------------------------------------------
# use current time as seed for every run
$defaultRNG seed 0


#   Application 
#------------------------------------------------------------------------
Agent/TCP               set packetSize_   $opt(PktSize)

# PHY
#------------------------------------------------------------------------
Phy/WirelessPhy set CPThresh_   10000.0         
Phy/WirelessPhy set CSThresh_   3.36463e-08     ;# 20% pkts sensed at 30m
Phy/WirelessPhy set RXThresh_   2.44547e-08     ;# 70% pkts received at 10m
Phy/WirelessPhy set Pt_         0.28183815      ;# watts

# MAC
#------------------------------------------------------------------------
Mac/802_11 set BER_               0.0000
Mac/802_11 set basicRate_         $opt(basicRate)
Mac/802_11 set dataRate_          $opt(dataRate)
Mac/802_11 set CWMin_             $opt(CWmin)
Mac/802_11 set CWMax_             $opt(CWmax)
Mac/802_11 set SlotTime_          $opt(SlotTime)
Mac/802_11 set SIFS_              $opt(SIFS)
Mac/802_11 set ShortRetryLimit_   $opt(ShortRetryLimit)
Mac/802_11 set LongRetryLimit_    $opt(LongRetryLimit)
Mac/802_11 set RTSThreshold_      $opt(RTSThreshold)
LL set totNumSTA_                 [expr $opt(nn)-1]

# create simulator instance
#------------------------------------------------------------------------
remove-all-packet-headers
add-packet-header TCP IP LL  Mac

set ns_   [new Simulator]
$ns_ at $opt(rectime) "print_stuff"
set tracefd  [open aaaa.tr w]
set namtrace [open aaaa.nam w]
$ns_ trace-all $tracefd
$ns_ namtrace-all-wireless $namtrace $opt(x) $opt(y)
set topo   [new Topography]
$topo load_flatgrid $opt(x) $opt(y)
create-god [expr $opt(nn)]

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
		 -agentTrace OFF \
                 -ifqTrace OFF \
                 -routerTrace OFF \
                 -macTrace OFF 

#create stations
#------------------------------------------------------------------------
for {set i 0} {$i < $opt(nn)} {incr i} {

    set node_($i) [$ns_ node]   

    $node_($i) set X_ [expr $i*30.0]
    $node_($i) set Y_ 0.0

    $node_($i) set Z_ 0.0
    $node_($i) random-motion 0          ;# disable random motion
}

set flowid 0
for {set i 0} {$i < $opt(fnum)} {incr i 1} {
    set tcp($flowid) [new Agent/TCP]
    set sink($flowid) [new Agent/TCPSink]
    if { $i == 0 } {
        $ns_ attach-agent $node_(0) $tcp($flowid)
        $ns_ attach-agent $node_(3) $sink($flowid)
    }
#     if { $i == 1 } {
#         $ns_ attach-agent $node_(0) $tcp($flowid)
#         $ns_ attach-agent $node_(3) $sink($flowid)
#     }
#     if { $i == 2 } {
#         $ns_ attach-agent $node_(0) $tcp($flowid)
#         $ns_ attach-agent $node_(3) $sink($flowid)
#     }
    if { $i == 1 } {
        $ns_ attach-agent $node_(0) $tcp($flowid)
        $ns_ attach-agent $node_(4) $sink($flowid)
    }
    if { $i == 2 } {
        $ns_ attach-agent $node_(5) $tcp($flowid)
        $ns_ attach-agent $node_(7) $sink($flowid)
    }
    $ns_ connect $tcp($flowid) $sink($flowid)

    $tcp($flowid) set fid_ $i

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# 1. set forwarder list manually, i.e., set the ids of nodes in the forwarder list
# 2. order matters, left has high priority
# 3. at least 2 hops, at most 5 hops
# For example, the following assigns 3 flows
#     if { $i == 0 } {
#         $tcp($flowid) ForwardList 0 1 2 3
#     }
#     if { $i == 1 } {
#         $tcp($flowid) ForwardList 0 1 2 4
#     }
#     if { $i == 2 } {
#         $tcp($flowid) ForwardList 5 6 1 7
#     }
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
#     if { $i == 0 } {
#         $tcp($flowid)  ForwardList 0 1 2 3
#         $sink($flowid) ForwardList 3 2 1 0
#     }
    if { $i == 0 } {
        $tcp($flowid)  ForwardList 0 3 -1 -1
        $sink($flowid) ForwardList 3 0 -1 -1
    }
#     if { $i == 0 } {
#         $tcp($flowid)  ForwardList 0 1 3 -1
#         $sink($flowid) ForwardList 3 1 0 -1
#     }
#     if { $i == 0 } {
#         $tcp($flowid)  ForwardList 0 2 3 -1
#         $sink($flowid) ForwardList 3 2 0 -1
#     }
    if { $i == 1 } {
        $tcp($flowid)  ForwardList 0 1 2 4
        $sink($flowid) ForwardList 4 2 1 0
    }
    if { $i == 2 } {
        $tcp($flowid)  ForwardList 5 6 1 7
        $sink($flowid) ForwardList 7 1 6 5
    }


    $tcp($flowid)  MacType 2;# // RIPPLE=0, LPR=1, SPR=2, ETX=2
    $sink($flowid) MacType 2
    

    set app($flowid) [new Application/FTP]
    $app($flowid) attach-agent $tcp($flowid)

    set f_cwnd  [open "out.spr.cwnd.flow.$flowid.txt" "w"]
    set f_throughput  [open "out.spr.thru.flow.$flowid.txt" "w"]
    set f_rtt  [open "out.spr.rtt.flow.$flowid.txt" "w"]
    close $f_cwnd 
    close $f_throughput 
    close $f_rtt 

    $ns_ at $opt(start) "$app($flowid) start" 
    set flowid [expr $flowid + 1]
}






# Result output routines
#########################################################################
$ns_ at $opt(stop) "stop"
$ns_ at $opt(stop) "$ns_ halt"

proc stop {} {
    global ns_ tracefd namtrace node_ opt  tcp sink  
        set now [$ns_ now]
        set flowid 0
        set tempsum 0.0
        set f_throughput  [open "results.multiflow.txt" "a"]
        puts $f_throughput "================================================="
        puts $f_throughput "SPR"
        puts $f_throughput "================================================="
        set aaa "+DATE(month/day/year): %m/%d/%Y, TIME: %H:%M:%S"
        set systemtime       "[exec date $aaa]"
        puts $f_throughput "$systemtime"
        puts $f_throughput "rate: $opt(rate) Mbps"
        puts $f_throughput "run: $opt(stop) seconds"
        puts $f_throughput "BER= $opt(BER)"
        puts $f_throughput "================================================="
        puts -nonewline  $f_throughput "flows: "
        for {set i 0} {$i < $opt(fnum)} {incr i} {
                set t_thru   [string range  [expr [$sink($flowid) set bytes_].0*8/1000000/$now] 0 6]
                set tempsum [expr $tempsum+$t_thru]
                puts -nonewline  $f_throughput  " [string range $t_thru 0 6] "
                set flowid [expr $flowid + 1]
        }
        puts $f_throughput "  sum: [string range $tempsum 0 6]"
        close $f_throughput 

    exec rm -rf aaaa.nam aaaa.tr
}


set step 0.01
proc print_stuff { } {
        global ns_ opt step tcp sink rdtcp rdsink
        set now [$ns_ now]

        set t_now  [string range $now 0 5]

        set flowid 0
        for {set i 0} {$i < $opt(fnum)} {incr i} {
                set t_cwnd   [string range [$tcp($flowid) set cwnd_] 0 4]
                set t_thru   [string range  [expr [$sink($flowid) set bytes_].0*8/1000000/$now] 0 6]
                set t_rtt    [string range  [$tcp($flowid) set srtt_] 0 4]

                set f_cwnd  [open "out.spr.cwnd.flow.$flowid.txt" "a"]
                set f_throughput  [open "out.spr.thru.flow.$flowid.txt" "a"]
                set f_rtt  [open "out.spr.rtt.flow.$flowid.txt" "a"]

                puts $f_cwnd  "$t_now $t_cwnd "
                puts $f_throughput  "$t_now $t_thru "
                puts $f_rtt  "$t_now $t_rtt "

                close $f_cwnd 
                close $f_throughput 
                close $f_rtt

                set flowid [expr $flowid + 1]
        }




        $ns_ at [expr $now + $step] "print_stuff"
}



puts "Starting Simulation..."
$ns_ run

